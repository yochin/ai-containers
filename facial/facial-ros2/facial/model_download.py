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
from tensorflow.keras.applications.mobilenet_v2 import preprocess_input
from tensorflow.keras.preprocessing.image import img_to_array
from tensorflow.keras.models import load_model
from retinaface import RetinaFaceDetector


class FacialModelDownloadNode(Node):

    def __init__(self):
        super().__init__('facial_model_download')

        path = "./facial_ros2/models"
        weightsPath = os.path.sep.join(
            [path, "Resnet50_Final.pth"])
        self.face_detector = RetinaFaceDetector(weightsPath, cpu=True)
        self.maskNet = load_model(
            os.path.sep.join([path, "mask_detector.model"]))


def main(args=None):
    rclpy.init(args=args)

    node = FacialModelDownloadNode()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
