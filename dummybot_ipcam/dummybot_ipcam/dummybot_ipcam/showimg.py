# import the necessary packages
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
import json
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from aai4r_edge_interfaces.msg import RobotImageInfo


class ImageShowNode(Node):
    def __init__(self):
        super().__init__('image_show')

        self.cv_bridge = CvBridge()
        '''
        self.subscription = self.create_subscription(
            Image,
            '/camera/robot_image',
            self.callback,
            10)
        self.subscription  # prevent unused variable warning
        '''
        self.subscription = self.create_subscription(
            RobotImageInfo,
            '/camera/robot_image_info', 
            self.ri_callback, 
            10)
        self.subscription  # prevent unused variable warning

        self.show = True


    def show_img(self, frame):
        if self.show:
            # show the frame and update the FPS counter
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)


    def callback(self, msg):
        print("called...")
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            print(cv_image.shape)
            self.show_img(cv_image)
        except CvBridgeError as e:
          print(e)        
      #np_arr = np.frombuffer(msg.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


    def ri_callback(self, msg):
        print("ri_called...")
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            print(json.loads(msg.params)['meal_start_time'])
            # cv_image = self.cv_bridge.imgmsg_to_cv2(msg.data, "bgr8")
            # print(cv_image.shape)
            self.show_img(cv_image)
        except CvBridgeError as e:
          print(e)        
      #np_arr = np.frombuffer(msg.data, np.uint8)
        #image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def main(args=None):
    rclpy.init(args=args)

    node = ImageShowNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
