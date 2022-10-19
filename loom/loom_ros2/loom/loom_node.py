import sys
import json
import time
import cv2
import numpy as np
import uuid

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

    def get_properties(self, subject):
        if subject not in self.memory:
            return None
        else:
            return self.memory[subject]

    def get_object(self, subject, property):
        if subject not in self.memory:
            return None
        if property not in self.memory[subject]:
            return None
        return self.memory[subject][property]

    def get_subject_ids(self):
        return self.memory.keys()

    def forget_chunk(self, cue):
        to_be_removed = []
        for subject in self.memory.keys():
            if 'related-to' in self.memory[subject] and self.memory[subject]['related-to'] == cue:
                to_be_removed.append(subject)
        for subject in to_be_removed:
            del self.memory[subject]


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
        self.timeline_lock = threading.Lock()

        self.agents = []
        self.agents_lock = threading.Lock()
        self.wms = {}
        self.wms_locks = {}

        #self.wm = WorkingMemory()
        #self.wm_lock = threading.Lock()

        # self.img_info = None
        # self.img_info_lock = threading.Lock()

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

        # self.subscription = self.create_subscription(
        #     String,
        #     '/aai4r/hhobjects',
        #     self.hhobjects_callback,
        #     10)

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


    def timer_callback(self):
        with self.timeline_lock:
            wms = self.wms
            self.timeline.add(self.wms)
            self.wms = {}
            with self.agents_lock:
                for id in self.agents:
                    self.wms[id] = WorkingMemory()
                    self.wms_locks[id] = threading.Lock()
        if len(wms) > 0:
            msg = self.generate_situation_msg(wms)
            self.publisher_.publish(msg)


    def add_agent(self, agent_id):
        if agent_id not in self.agents:
            self.agents.append(agent_id)


    def get_wm(self, agent_id):
        if agent_id in self.wms:
            return self.wms[agent_id], self.wms_locks[agent_id]
        else:
            return None, None


    def publish_img(self, img):
        self.monitor_publisher.publish(
            self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def detrack_callback(self, msg):
        msg_data = json.loads(msg.data)
        # get a world model for the current agent
        agent_id = msg_data['agent_id']
        wm, lock = self.get_wm(agent_id)
        if wm is None or lock is None:
            print('no wm for agent {}'.format(agent_id))
            return
        with lock:
            for track in msg_data['tracks']:
                wm.put(track['track_id'], 'roi', track['pos'])
                wm.put(track['track_id'], 'score', track['score'])


    def facial_callback(self, msg):
        msg_data = json.loads(msg.data)

        # get a world model for the current agent
        agent_id = msg_data['agent_id']
        wm, lock = self.get_wm(agent_id)

        if wm is None:
            return

        with lock:
            if len(msg_data['facial']) > 0:
                for face in msg_data['facial']:
                    wm.put('robot', 'face_detected', 'yes')
                    track_id = self.predict_track_id(wm, face['face_bbox'])
                    if track_id is None:
                        continue
                    wm.put(track_id, 'gender', face['gender'][0])
                    wm.put(track_id, 'age', (int)(face['age']))
                    wm.put(track_id, 'mask', face['mask'][0])
                    wm.put(track_id, 'hair_style', face['hair_style'])
                    wm.put(track_id, 'hair_color', face['hair_color'])
                    wm.put(track_id, 'hair_length', face['hair_length'])
                    wm.put(track_id, 'nationality', face['nationality'])
                    wm.put(track_id, 'face_roi', face['face_bbox'])
            else:
                wm.put('robot', 'face_detected', 'no')


    def meal_callback(self, msg):
        msg_data = json.loads(msg.data)
        
        # get a world model for the current agent
        agent_id = msg_data['agent_id']
        wm, lock = self.get_wm(agent_id)

        if wm is None:
            return

        with lock:
            wm.forget_chunk('meal-context')
            for object in msg_data['meal']:
                # object = [{'category': 'food', 'name': 'pasta', 'bbox': (100,100,200,200), 'amount': 0.9}]
                obj_id = self.get_uuid()
                wm.put(obj_id, 'related-to', 'meal-context')
                wm.put(obj_id, 'category', object['category'])
                wm.put(obj_id, 'name', object['name'])
                if 'amount' in object:
                    wm.put(obj_id, 'amount', object['amount'])


    # def hhobjects_callback(self, msg):
    #     msg_data = json.loads(msg.data)
    #     with self.wm_lock:
    #         for object in msg_data['hhobjects']:
    #             # object = [(100,100,200,200), 0.787, 'cup']]
    #             track_id = self.predict_track_id(object[0])
    #             if track_id is None:
    #                 continue
    #             if object[2] == 'cup':
    #                 self.wm.put(track_id, 'cup_roi', object[0])
    #                 self.wm.put(track_id, 'has_cup', True)


    def image_info_callback(self, msg):
        #stamp = msg.stamp
        #now = self.get_clock().now().to_msg()
        #nano_diff = stamp.nanosec - now.nanosec
        # if abs(nano_diff) > 30000000:
        #    return
        self.add_agent(msg.agent_id)
        wm, lock = self.get_wm(msg.agent_id)

        if wm is None:
            return

        with lock:
            wm.put('robot', 'robot_id', msg.agent_id)
            wm.put('robot', 'distance', msg.distance)
            wm.put('robot', 'zone', msg.zone)

        '''
        np_arr = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # put an image for processing
        self.image_q.put(im)

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
        '''


    def get_uuid(self):
        return str(uuid.uuid4())


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


    def predict_track_id(self, wm, face_roi):
        #return '12345'
        subjects = wm.get_subject_ids()
        for key in subjects:
            track = wm.get_properties(key)
            if track is None:
                continue
            track_box = self.get_box(track)
            if track_box is None:
                continue
            if self.calc_iou(track_box, face_roi) > 0.7:
                self.get_logger().debug("IOU: %f" % self.calc_iou(track_box, face_roi))
                return key


    def generate_situation_msg(self, wms):
        situations = []
        print(wms.keys())
        for agent_id in wms.keys():
            wm = wms[agent_id]
            situation = {
                'domain': 1,
                'robot_id': agent_id,
                'robot_type': 1,
                'time': datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
                'zone': 0,
                'distance': -1,
                'situation': {},
                'group': {},
                'personal_context': [],
                'meal_context': []      # [{'category':'food', 'name':'pasta', 'amount':0.2}, ...]
            }
            personal_attributes = ['mask', 'expression', 'gender', 'age', 'hair_style', 'hair_color', 'hair_length', 'nationality', 'skin_color']
            for key in wm.memory.keys():
                track = wm.memory[key]
                if key == 'robot':
                    situation.update(track)
                else:
                    context = {}
                    if 'related-to' in track and track['related-to'] == 'meal-context':
                        context['category'] = track['category']
                        context['name'] = track['name']
                        context['amount'] = track['amount']
                        situation['meal_context'].append(context)
                    else:
                        context['id'] = key
                        for attr in personal_attributes:
                            if attr in track:
                                context[attr] = track[attr]
                        # if 'mask' in track:
                        #     context['mask'] = 1 if track['mask'] is 1 else 2
                        # if 'has_cup' in track:
                        #     context['has_cup'] = 1 if track['has_cup'] is True else 2                    
                        situation['personal_context'].append(context)
            
            situations.append(situation)

        msg = String()
        msg.data = json.dumps(situations)
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


def main(args=None):
    rclpy.init(args=args)

    node = LoomNode()

    rclpy.spin(node)

    node.release_video_writer()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
