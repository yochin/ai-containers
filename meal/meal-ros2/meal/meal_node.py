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

from food_detector_interface import FoodDetectionRequestHandler

class MealNode(Node):
    def __init__(self):
        super().__init__('meal_node')

        # Prepare a food detector
        model_path = '/aai4r/aai4r-ServiceContextUnderstanding/output'
        self.meal_detector = FoodDetectionRequestHandler(model_path)

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
        self.monitor_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def visualize(self, frame, meal_dets):
        return frame


    def get_uuid(self):
        return str(uuid.uuid4())


    def callback(self, msg):
        self.get_logger().info("meal_node callback!")
        stamp = msg.stamp
        now = self.get_clock().now().to_msg()
        nano_diff = stamp.nanosec - now.nanosec
        if abs(nano_diff) > 50000000:
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        agent_id = msg.agent_id

        #frame = imutils.resize(im, width=400)

        results, vis_img = self.meal_detector.process_inference_request(frame)
        
        # meal context for an agent(robot)
        msg_data = {"agent_id": agent_id, "timestamp":(stamp.sec,stamp.nanosec), "meal": []}

        for result in results:
            meal_context = {}
            meal_context['category'] = result[4]
            meal_context['name'] = result[6]
            meal_context['bbox'] = [result[0],result[1],result[2],result[3]]
            if len(result) >= 8:
                meal_context['amount'] = result[7]
            msg_data['meal'].append(meal_context)        

        # publish message
        self.get_logger().info(json.dumps(msg_data))

        pub_msg = String()
        pub_msg.data = json.dumps(msg_data)
        self.publisher_.publish(pub_msg)

        if self.visualize_flag:
            self.publish_img(self.visualize(vis_img, []))


def main(args=None):
    rclpy.init(args=args)

    node = MealNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
