# import the necessary packages
from imutils.video import WebcamVideoStream
from imutils.video import FPS
import numpy as np
import imutils
import time
import cv2
import json
import os
import datetime
import rtsp

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from aai4r_edge_interfaces.msg import RobotImageInfo

# walk a file at a time in a directory
def walk_dir(path):
    for root, dirs, files in os.walk(path):
        dirs.sort()
        for file in sorted(files):
            yield os.path.join(root, file)

def get_time_of_file(file):
    x = file.split('/')[-1].split('_')[-1].split('.')[0].split('-')
    x = x[:-1]
    x = '-'.join(x)
    t = datetime.datetime.strptime(x, "%Y-%m-%d-%H-%M-%S")
    return x, t

class CameraNode(Node):
    def __init__(self):
        super().__init__('ip_camera_node')

        self.cv_bridge = CvBridge()

        self.publisher = self.create_publisher(RobotImageInfo, '/camera/robot_image_info', 1)
        self.img_publisher = self.create_publisher(Image, '/camera/robot_image', 1)

        self.publisher_meal_event = self.create_publisher(String, '/meal/event', 1)

        self.ip_addr_1 = 'rtsp://admin:1234567@192.168.50.146:554/stream1'
        self.ip_addr_2 = 'rtsp://admin:1234567@192.168.50.30:554/stream1'
        self.ip_addr_3 = 'rtsp://admin:1234567@192.168.50.3:554/stream1'

        self.cap = rtsp.Client(rtsp_server_uri=self.ip_addr_1, verbose=True)

        time.sleep(1.0)
        
        self.stopped = False

        self.show = False

        self.create_timer(0.02, self.perform)

        self.count = 0
        self.agents = ['robot01']


    def show_img(self, frame):
        if self.show:
            # show the frame and update the FPS counter
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)


    def perform(self):
        frame = self.cap.read()

        if frame is not None:
            # frame_resized = frame1.resize((int(frame1.width / 4), int(frame1.height / 4)))
            '''
            frame_resized = frame
            cv_im = np.array(frame_resized)
            cv_im_bgr = cv_im[:, :, ::-1].copy()  # rgb to bgr
            '''

            frame = np.array(frame)
            frame = frame[:, :, ::-1].copy()  # rgb to bgr
            self.publish(frame, -1)
            self.publish_img(frame)
            self.count = (self.count + 1) % len(self.agents)
            self.get_logger().info('published...')

            if self.show:
                self.show_img(frame)


    def publish_img(self, img):
        self.img_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def publish(self, frame, meal_current_time):
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
        #msg.params = json.dumps({'meal_current_time': meal_current_time})
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
