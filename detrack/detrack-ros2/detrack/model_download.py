import sys
import json
import numpy as np

import rclpy
from rclpy.node import Node

from detrack.detracker import Detracker

from std_msgs.msg import String
from sensor_msgs.msg import Image as SensorImage


class DetrackNode(Node):

    def __init__(self):
        super().__init__('model_download')

        self.tracker = Detracker()


def main(args=None):
    rclpy.init(args=args)

    node = DetrackNode()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
