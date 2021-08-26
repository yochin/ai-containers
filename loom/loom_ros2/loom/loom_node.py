import sys
import json
import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from aai4r_edge_interfaces.msg import RobotImageInfo

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import threading
import copy
from datetime import datetime
import os
from queue import Queue

from loom.visualization import ObjectResult, draw_labeled_boxes


class WorkingMemory(object):
    def __init__(self):
        super().__init__()
        # memory is indexed first by subject, and then by propety
        # {
        #  "person01": {"age": 19, "name": "Minsu"},
        #  "person02": {"age": 50, "name": "Jackson"},
        #  ...
        # }
        self.memory = {}

    def put(self, subject, property, object):
        if subject not in self.memory:
            self.memory[subject] = {}
        self.memory[subject][property] = object

    def get(self, subject):
        if subject not in self.memory:
            return None
        else:
            return self.memory[subject]

    def get(self, subject, property):
        if subject not in self.memory:
            return None
        if property not in self.memory[subject]:
            return None
        return self.memory[subject][property]


class CognitiveTimeline(object):
    def __init__(self) -> None:
        super().__init__()
        self.timeline = []
        self.max_size = 1000

    def add(self, snapshot):
        self.timeline.append(snapshot)
        if len(self.timeline) > self.max_size:
            del self.timeline[0]


class LoomNode(Node):
    def __init__(self):
        super().__init__('loom_node')

        self.timeline = CognitiveTimeline()

        self.wm = WorkingMemory()
        self.wm_lock = threading.Lock()

        self.img_info = None
        self.img_info_lock = threading.Lock()

        self.subscription = self.create_subscription(
            String,
            '/aai4r/detrack',
            self.detrack_callback,
            10)

        self.subscription = self.create_subscription(
            String,
            '/aai4r/facial',
            self.facial_callback,
            10)

        self.subscription = self.create_subscription(
            String,
            '/aai4r/meal',
            self.meal_callback,
            10)

        self.subscription = self.create_subscription(
            String,
            '/aai4r/hhobjects',
            self.hhobjects_callback,
            10)

        self.subscription = self.create_subscription(
            RobotImageInfo,
            '/camera/robot_image_info',
            self.image_info_callback,
            1)
        self.subscription  # prevent unused variable warning

        # Output Topic: Situation Information
        self.publisher_ = self.create_publisher(String, '/aai4r/situation', 10)

        self.vizualize_flag = True
        self.monitor_publisher = self.create_publisher(
            Image, '/aai4r/loom/monitor', 1)
        self.cv_bridge = CvBridge()

        self.create_timer(0.5, self.timer_callback)

        self.image_q = Queue()
        self.prev_image_frame = None
        self.video_writer = None
        self.image_size = (640, 480)
        self.fps = 30.0
        self.time_video_recording_started = None
        self.create_timer(1.0/self.fps, self.video_recording_callback)

    def publish_img(self, img):
        self.monitor_publisher.publish(
            self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))

    def detrack_callback(self, msg):
        msg_data = json.loads(msg.data)
        with self.wm_lock:
            for track in msg_data['tracks']:
                self.wm.put(track['track_id'], 'roi', track['pos'])
                self.wm.put(track['track_id'], 'score', track['score'])

    def facial_callback(self, msg):
        msg_data = json.loads(msg.data)
        with self.wm_lock:
            if len(msg_data['facial']) > 0:
                for face in msg_data['facial']:
                    self.wm.put('robot', 'face_detected', 'yes')
                    track_id = self.predict_track_id(face['box'])
                    if track_id is None:
                        continue
                    self.wm.put(track_id, 'mask', face['mask'])
                    self.wm.put(track_id, 'face_roi', face['box'])
            else:
                self.wm.put('robot', 'face_detected', 'no')

    def hhobjects_callback(self, msg):
        msg_data = json.loads(msg.data)
        with self.wm_lock:
            for object in msg_data['hhobjects']:
                # object = [(100,100,200,200), 0.787, 'cup']]
                track_id = self.predict_track_id(object[0])
                if track_id is None:
                    continue
                if object[2] == 'cup':
                    self.wm.put(track_id, 'cup_roi', object[0])
                    self.wm.put(track_id, 'has_cup', True)

    def image_info_callback(self, msg):
        #stamp = msg.stamp
        #now = self.get_clock().now().to_msg()
        #nano_diff = stamp.nanosec - now.nanosec
        # if abs(nano_diff) > 30000000:
        #    return
        np_arr = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # put an image for processing
        self.image_q.put(im)

        with self.img_info_lock:
            self.img_info = msg

        if self.vizualize_flag:
            results = []
            for key in self.wm.memory.keys():
                mem_item = self.wm.memory[key]
                if 'roi' in mem_item:
                    results.append(ObjectResult(
                        key, str(key), 0.9, mem_item['roi']))
                if 'face_roi' in mem_item:
                    results.append(ObjectResult(
                        key, 'face', 0.9, mem_item['face_roi']))
                if 'cup_roi' in mem_item:
                    results.append(ObjectResult(
                        key, 'cup', 0.9, mem_item['cup_roi']))

            imp = draw_labeled_boxes(im, results)
            self.publish_img(imp)

    def meal_callback(self, msg):
        msg_data = json.loads(msg.data)
        # TODO: implement...

    def get_box(self, track):
        if 'roi' in track:
            return track['roi']
        else:
            return None

    def calc_iou(self, track_roi, face_roi):
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(track_roi[0], face_roi[0])
        yA = max(track_roi[1], face_roi[1])
        xB = min(track_roi[2], face_roi[2])
        yB = min(track_roi[3], face_roi[3])
        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
        # compute the area of face_roi
        face_area = (face_roi[2] - face_roi[0] + 1) * \
            (face_roi[3] - face_roi[1] + 1)
        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        containment = interArea / float(face_area)
        # return the intersection over union value
        return containment

    def predict_track_id(self, face_roi):
        for key in self.wm.memory.keys():
            track = self.wm.memory[key]
            track_box = self.get_box(track)
            if track_box is None:
                return None
            if self.calc_iou(track_box, face_roi) > 0.7:
                self.get_logger().debug("IOU: %f" % self.calc_iou(track_box, face_roi))
                return key

    def generate_situation_msg(self, wm):
        situation = {
            'domain': 1,
            'robot_id': 'robot01',
            'robot_type': 1,
            'time': datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
            'zone': 0,
            'distance': -1,
            'situation': {},
            'group': {},
            'personal_context': []
        }

        for key in wm.memory.keys():
            track = wm.memory[key]
            if key == 'robot':
                situation.update(track)
            else:
                context = {}
                context['id'] = key
                if 'mask' in track:
                    context['mask'] = 1 if track['mask'] is True else 2
                if 'has_cup' in track:
                    context['has_cup'] = 1 if track['has_cup'] is True else 2
                situation['personal_context'].append(context)

        '''
        string agent_id	# ex) hancom_qi_1
        string format		# jpeg
        string hash		# image hash
        uint64 seq_id		# tick count
        uint16 height		# 640
        uint16 width		# 480
        float32 distance	# Meter
        uint8 zone		# 1.Entrance 2.Lobby 3.table
        uint8[] data		# Compressed image buffer
        '''
        with self.img_info_lock:
            if self.img_info is not None:
                situation['robot_id'] = self.img_info.agent_id
                situation['distance'] = self.img_info.distance
                situation['zone'] = self.img_info.zone

        msg = String()
        msg.data = json.dumps(situation)
        self.get_logger().info("SITUATION: %s" % msg.data)
        return msg


    def create_video_writer(self):
        video_file_name = datetime.now().strftime("%Y-%m-%dT%H-%M-%S") + '.avi'
        video_file_path = os.path.join("/video_backup", video_file_name)
        four_cc = cv2.VideoWriter_fourcc(*'XVID')
        #four_cc =  cv2.VideoWriter_fourcc('X', '2', '6', '4')
        video_writer = cv2.VideoWriter(
            video_file_path, four_cc, 
            self.fps, self.image_size)
        return video_writer


    def video_recording_callback(self):
        if self.video_writer is None:
            self.video_writer = self.create_video_writer()
            self.time_video_recording_started = time.time()
        elif time.time() >= (self.time_video_recording_started + 3600):
            self.video_writer.release()
            self.video_writer = self.create_video_writer()
            self.time_video_recording_started = time.time()

        if self.image_q.empty():
            if self.prev_image_frame is not None:
                self.video_writer.write(self.prev_image_frame)
        else:
            #while not self.image_q.empty():
            img = self.image_q.get()
            self.image_q.task_done()
            if img is not None:
                self.prev_image_frame = img
                self.video_writer.write(img)


    def release_video_writer(self):
        if self.video_writer is not None:
            self.video_writer.release()


    def timer_callback(self):
        with self.wm_lock:
            wm = self.wm
            self.timeline.add(self.wm)
            self.wm = WorkingMemory()
        msg = self.generate_situation_msg(wm)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = LoomNode()

    rclpy.spin(node)

    node.release_video_writer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
