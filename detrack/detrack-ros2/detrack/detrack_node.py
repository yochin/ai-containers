import sys
import json
import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from detrack.detracker import Detracker

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from aai4r_edge_interfaces.msg import RobotImageInfo

class DetrackNode(Node):

    def __init__(self):
        super().__init__('detrack_node')

        self.trackers = {}

        self.subscription = self.create_subscription(
            RobotImageInfo,
            '/camera/robot_image_info',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

        self.show_image = True

        self.publisher_ = self.create_publisher(String, '/aai4r/detrack', 1)

        self.monitor_publisher = self.create_publisher(Image, '/aai4r/detrack/monitor', 1)
        self.cv_bridge = CvBridge()


    def add_agent(self, agent_id):
        if agent_id not in self.trackers:
            self.trackers[agent_id] = Detracker()


    def callback(self, msg):
        stamp = msg.stamp
        now = self.get_clock().now().to_msg()
        nano_diff = stamp.nanosec - now.nanosec
        if abs(nano_diff) > 50000000:
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        im = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        agent_id = msg.agent_id
        self.add_agent(agent_id)
        
        tracker = self.trackers[msg.agent_id]

        tracks, detections, frame = tracker.process_frame(im)

        trks = []
        for trk in tracks:
            trks.append({'track_id':trk.id, 'pos':list(trk.box), 'score':trk.score})
            if self.show_image:
                im = cv2.rectangle(im, (int(trk.box[0]),int(trk.box[1])), (int(trk.box[2]),int(trk.box[3])), (255,0,0), 2)

        if self.show_image is True:
            self.publish_img(im)

        msg = String()
        msg.data = json.dumps({"timestamp":(stamp.sec,stamp.nanosec), "agent_id":agent_id, "tracks": trks})
        #self.get_logger().info('track: %s' % msg.data)
        self.publisher_.publish(msg)

    def publish_img(self, img):
        self.monitor_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))
        

def main(args=None):
    rclpy.init(args=args)

    node = DetrackNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
