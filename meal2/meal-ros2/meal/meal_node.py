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

        self.publisher_ = self.create_publisher(String, '/aai4r/meal', 10)

        self.visualize_flag = True

        self.monitor_publisher = self.create_publisher(Image, '/aai4r/meal/monitor', 1)

        self.cv_bridge = CvBridge()


    def publish_img(self, img):
        img = np.array(img) # convert to numpy array
        self.monitor_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def visualize(self, frame, meal_dets):
        return frame


    def get_uuid(self):
        return str(uuid.uuid4())


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

        t = time.time()
        if msg.params is not None and msg.params != '':
            d = json.loads(msg.params)
            if d['meal_start_time'] is not None:
                t = d['meal_start_time']
            else:
                print('meal_start_time is None')
                return
        else:
            print('msg.params is None')
            return

        #frame = imutils.resize(im, width=400)
        frame = PIL.Image.fromarray(frame)

        # 1. Perform detection and classification
        # - detection_result is a list of [x1,y1,x2,y2,class_id]
        # - ex) result = [(100,100,200,200,154), (200,300,200,300,12)]
        # - service_result is a list of four service possible time (food refill, trash collection, serving dessert, lost item)
        # - ex) result = [0.7, 0.1, 0.1, 0.2]
        detection_results, service_results, im2show = \
            self.meal_detector.process_inference_request(frame, int(time.time() - t))

        # meal context for an agent(robot)
        msg_data = {"agent_id": agent_id, "timestamp":(stamp.sec,stamp.nanosec), "meal": [], 'meal_event': ''}

        if detection_results is None or len(detection_results) == 0:
            print("no detection results")
            return

        for result in detection_results:
            meal_context = {}
            meal_context['category'] = result[4]
            meal_context['bbox'] = [result[0],result[1],result[2],result[3]]
            msg_data['meal'].append(meal_context)     

        # get the most possible meal event
        msg_data['meal_event'] = service_results.index(max(service_results))

        # publish message
        self.get_logger().info(json.dumps(msg_data))

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
