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


class MealModelDownloadNode(Node):

    def __init__(self):
        super().__init__('meal_model_download')


def main(args=None):
    rclpy.init(args=args)

    node = MealModelDownloadNode()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
