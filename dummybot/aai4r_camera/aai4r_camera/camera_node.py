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
        super().__init__('camera_node')

        self.cv_bridge = CvBridge()

        self.publisher = self.create_publisher(RobotImageInfo, '/camera/robot_image_info', 1)
        self.img_publisher = self.create_publisher(Image, '/camera/robot_image', 1)

        self.publisher_meal_event = self.create_publisher(String, '/meal/event', 1)

        self.use_webcam = False

        if self.use_webcam:
            self.fvs = WebcamVideoStream(0).start()
            time.sleep(1.0)
        
        # open a directory at a path
        self.path = '/aai4r/samples'
        self.notify_meal_start_time()
        time.sleep(1)
        self.notify_meal_start_time()
        time.sleep(1)
        self.notify_meal_start_time()
        time.sleep(1)
        self.notify_meal_start_time()
        time.sleep(1)
        self.files = walk_dir(self.path)
        print('start time is {}'.format(self.meal_start_time))

        self.stopped = False

        self.show = False

        self.create_timer(1, self.perform)

        self.img = cv2.imread('/aai4r/example.jpg')

        self.count = 0
        self.agents = ['robot01']


    def notify_meal_start_time(self):
        time_in_string, self.meal_start_time = get_time_of_file('/aai4r/samples/captured_2023-01-18-11-50-07-181543.jpg')
        msg = String()
        #msg.data = json.dumps({'meal_start_time': time_in_string})
        msg.data = json.dumps({'meal_start_time': ''})
        self.publisher_meal_event.publish(msg)
        print('published meal start time: {}'.format(time_in_string))


    def show_img(self, frame):
        if self.show:
            # show the frame and update the FPS counter
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)


    def perform(self):
        if self.use_webcam:
            frame = self.fvs.read()
            meal_current_time = 10
        else:
            while True:
                file = next(self.files)
                if file.endswith('.jpg'):
                    print(file)
                    img = cv2.imread(file)
                    x, t = get_time_of_file(file)
                    d = t - self.meal_start_time
                    # timedelta into seconds
                    print("{} seconds passed".format(d.total_seconds()))
                    frame = img
                    meal_current_time = x
                    break

        self.publish(frame, meal_current_time)
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
        msg.params = json.dumps({'meal_current_time': meal_current_time})
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
