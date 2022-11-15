# import the necessary packages
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from aai4r_edge_interfaces.msg import RobotImageInfo

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.cv_bridge = CvBridge()

        self.publisher = self.create_publisher(RobotImageInfo, '/camera/robot_image_info', 1)
        self.img_publisher = self.create_publisher(Image, '/camera/robot_image', 1)

        self.fvs = WebcamVideoStream(0).start()
        time.sleep(1.0)

        self.stopped = False

        self.show = False

        self.create_timer(0.02, self.perform)

        self.count = 0
        self.agents = ['robot01', 'robot02', 'robot03']

    def show_img(self, frame):
        if self.show:
            # show the frame and update the FPS counter
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)


    def perform(self):
        frame = self.fvs.read()

        self.publish(frame)
        self.publish_img(frame)
        self.count = (self.count + 1) % len(self.agents)
        self.get_logger().info('published...')

        if self.show:
            self.show_img(frame)


    def publish_img(self, img):
        self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def publish(self, frame):
        msg = RobotImageInfo()
        msg.stamp = self.get_clock().now().to_msg()
        msg.agent_id = self.agents[0]
        msg.format = 'jpg'
        msg.hash = ''
        msg.seq_id = 0
        msg.height = 0
        msg.width = 0
        msg.distance = 0.12
        msg.zone = 2
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        self.publisher.publish(msg)


    def stop(self):
        self.stopped = True
        cv2.destroyAllWindows()
        #fvs.stop()


def main(args=None):
    rclpy.init(args=args)

    node = CameraNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
