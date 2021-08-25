import sys
import json
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from aai4r_edge_interfaces.msg import RobotImageInfo

import os
import cv2
import imutils

class MealNode(Node):
    def __init__(self):
        super().__init__('meal_node')

        self.meal_detector = None

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


    def callback(self, msg):
        stamp = msg.stamp
        now = self.get_clock().now().to_msg()
        nano_diff = stamp.nanosec - now.nanosec
        if abs(nano_diff) > 50000000:
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        agent_id = msg.agent_id

        #frame = imutils.resize(im, width=400)

        # TODO: detect or recognize something

        msg_data = {"timestamp":(stamp.sec,stamp.nanosec), "facial": []}
        for (box, pred) in zip(face_locs, mask_dets):
            # print("FACE: ", box, " / MASK: ",
            #      "ON" if pred[0] > pred[1] else "OFF")
            msg_data["facial"].append(
                {"box": (int(box[0]), int(box[1]), int(box[2]), int(box[3])),
                "mask": bool(pred[0] > pred[1])})
        #self.get_logger().info(json.dumps(msg_data))
        pub_msg = String()
        pub_msg.data = json.dumps(msg_data)
        self.publisher_.publish(pub_msg)

        if self.visualize_flag:
            self.publish_img(self.visualize(frame, []))


def main(args=None):
    rclpy.init(args=args)

    node = MealNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
