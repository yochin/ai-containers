import sys
import json
import numpy as np
import uuid

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from aai4r_edge_interfaces.msg import RobotImageInfo

import os
import cv2
import imutils
import time
import PIL
import datetime

from table_service_alarm_interface import TableServiceAlarmRequestHandler


class MealNode(Node):
    def __init__(self):
        super().__init__('meal_node')

        # Prepare a food detector
        model_path = '/aai4r/aai4r-TableServiceDetection'
        self.meal_detector = TableServiceAlarmRequestHandler(model_path)

        # subscribe to image inputs
        self.subscription = self.create_subscription(
            RobotImageInfo,
            '/camera/robot_image_info',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

        self.meal_start_time = None
        self.subscription = self.create_subscription(
            String,
            '/meal/event',
            self.callback_meal_event,
            1)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, '/aai4r/meal', 10)

        self.visualize_flag = True
        self.image_backup_flag = True
        
        self.monitor_publisher = self.create_publisher(Image, '/aai4r/meal/monitor', 1)

        self.cv_bridge = CvBridge()


    def publish_img(self, img):
        img = np.array(img) # convert to numpy array
        self.monitor_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def visualize(self, frame, meal_dets):
        return frame


    def get_uuid(self):
        return str(uuid.uuid4())


    def callback_meal_event(self, msg):
        self.get_logger().info("meal_node callback_meal_event!")
        msg_data = json.loads(msg.data)
        if msg_data['meal_start_time'] is not None:
            self.meal_detector.process_start_meal(0)
            if msg_data['meal_start_time'] == '':
                self.meal_start_time = datetime.datetime.now()
            else:
                self.meal_start_time = datetime.datetime.strptime(msg_data['meal_start_time'],"%Y-%m-%d-%H-%M-%S")
        elif msg_data['meal_end_time'] is not None:
            self.meal_start_time = None


    def callback(self, msg):
        self.get_logger().info("meal_node callback!")
        stamp = msg.stamp
        now = self.get_clock().now().to_msg()
        nano_diff = now.nanosec - stamp.nanosec
        #if abs(nano_diff) > 50000000:
        #    return
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        agent_id = msg.agent_id

        if self.meal_start_time is None:
            self.get_logger().info("meal start time is None")
            return

        d = json.loads(msg.params)
        t = datetime.datetime.now()
        meal_duration = (t - self.meal_start_time).total_seconds()
        # self.get_logger().info('meal_start_time: {}'.format(self.meal_start_time))
        # self.get_logger().info('current time: {}'.format(t))
        # self.get_logger().info('meal_duration: {}'.format(meal_duration))

        #frame = imutils.resize(im, width=400)
        frame = PIL.Image.fromarray(frame)

        # 1. Perform detection and classification
        # - detection_result is a list of [x1,y1,x2,y2,class_id]
        # - ex) result = [(100,100,200,200,154), (200,300,200,300,12)]
        # - service_result is a list of four service possible time (food refill, trash collection, serving dessert, lost item)
        # - ex) result = [0.7, 0.1, 0.1, 0.2]
        # - repr_service_index and repr_service_name
        # : (0) no_service, (1) refill_food, (2) found_trash, (3) provide_dessert, (4) found_lost
        # - ex) result = 0, no_service
        detection_results, service_results, repr_service_index, repr_service_name, im2show = \
            self.meal_detector.process_inference_request(frame, int(meal_duration))

        # meal context for an agent(robot)
        msg_data = {"agent_id": agent_id, "timestamp":(stamp.sec,stamp.nanosec), "meal": [], 'meal_event': ''}

        if detection_results is None or len(detection_results) == 0:
            # print("no detection results")
            return

        # print time info when the result is available
        self.get_logger().info('meal_start_time: {}'.format(self.meal_start_time))
        self.get_logger().info('current time: {}'.format(t))
        self.get_logger().info('meal_duration: {}'.format(meal_duration))


        for result in detection_results:
            meal_context = {}
            meal_context['category'] = result[4]
            meal_context['bbox'] = [result[0],result[1],result[2],result[3]]
            msg_data['meal'].append(meal_context)     

        # get the most possible meal event
        self.get_logger().info("service_results = {}".format(service_results))

        msg_data['meal_event'] = repr_service_index

        # publish message
        self.get_logger().info("meal event = {}".format(msg_data['meal_event']))
        #self.get_logger().info(json.dumps(msg_data))

        if self.image_backup_flag:
            file_name = datetime.now().strftime("%Y-%m-%dT%H-%M-%S.%f") + '_{}'.format(msg_data['meal_event']) + '.jpg'
            file_path = os.path.join("/aai4r/captures", file_name)
            frame.save(file_path)

        pub_msg = String()
        pub_msg.data = json.dumps(msg_data)
        self.publisher_.publish(pub_msg)

        if self.visualize_flag:
            self.publish_img(self.visualize(im2show, []))


def main(args=None):
    rclpy.init(args=args)

    node = MealNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
