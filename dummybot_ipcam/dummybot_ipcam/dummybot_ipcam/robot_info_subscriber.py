# import the necessary packages
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2

import rclpy
from rclpy.node import Node

from aai4r_edge_interfaces.msg import RobotImageInfo


class RobotInfoSubscriberNode(Node):
    def __init__(self):
        super().__init__('robot_info_subscriber')

        self.subscription = self.create_subscription(
            RobotImageInfo,
            '/camera/robot_image_info',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning

        self.show = True

    def show_img(self, frame):
        if self.show:
            # show the frame and update the FPS counter
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)

    def callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.show_img(image_np)

def main(args=None):
    rclpy.init(args=args)

    node = RobotInfoSubscriberNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
