# import the necessary packages
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image


class ImageShowNode(Node):
    def __init__(self):
        super().__init__('image_show')

        self.cv_bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/aai4r/detrack_monitor',
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
        print("called...")
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            print(cv_image.shape)
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
