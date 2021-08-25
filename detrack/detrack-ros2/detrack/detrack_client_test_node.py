#
#
import sys
import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image as SensorImage


class DetrackClient(Node):
    def __init__(self):
        super().__init__('detrack_client')
        self.subscription = self.create_subscription(
            String,
            '/aai4r/detrack',
            self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info('I have seen: "%s"' % msg.data)

def main(args=None):
    print("version is ", sys.version)
    rclpy.init(args=args)

    image_publisher = DetrackClient()
    rclpy.spin(image_publisher)

    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
