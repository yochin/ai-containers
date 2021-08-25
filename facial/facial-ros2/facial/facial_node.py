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


class FaceMaskDetector(object):
    def __init__(self, logger) -> None:
        super().__init__()

        path = "./facial_ros2/models"
        weightsPath = os.path.sep.join(
            [path, "Resnet50_Final.pth"])
        self.face_detector = RetinaFaceDetector(weightsPath)
        self.maskNet = load_model(
            os.path.sep.join([path, "mask_detector.model"]))

        self.confidence = 0.8
        self.logger = logger


    def detect_and_predict_mask(self, frame):
        # grab the dimensions of the frame and then construct a blob
        # from it
        # (h, w) = frame.shape[:2]
        # blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300),
        #                              (104.0, 177.0, 123.0))

        # pass the blob through the network and obtain the face detections
        # self.faceNet.setInput(blob)
        # detections = self.faceNet.forward()

        dets = self.face_detector.detect(frame)

        # initialize our list of faces, their corresponding locations,
        # and the list of predictions from our face mask network
        faces = []
        locs = []
        preds = []

        # loop over the detections
        for det in dets:
            startX, startY, endX, endY, confidence = int(det[0]), int(det[1]), int(det[2]), int(det[3]), float(det[4])
            #self.logger.info('x1=%d y1=%d x2=%d y2=%d conf=%f' % (startX, startY, endX, endY, confidence))
            # filter out weak detections by ensuring the confidence is
            # greater than the minimum confidence
            if confidence > self.confidence:
                # extract the face ROI, convert it from BGR to RGB channel
                # ordering, resize it to 224x224, and preprocess it
                face = frame[startY:endY, startX:endX]
                face = cv2.cvtColor(face, cv2.COLOR_BGR2RGB)
                face = cv2.resize(face, (224, 224))
                face = img_to_array(face)
                face = preprocess_input(face)

                # add the face and bounding boxes to their respective
                # lists
                faces.append(face)
                locs.append((startX, startY, endX, endY))

        # only make a predictions if at least one face was detected
        if len(faces) > 0:
            # for faster inference we'll make batch predictions on *all*
            # faces at the same time rather than one-by-one predictions
            # in the above `for` loop
            faces = np.array(faces, dtype="float32")
            preds = self.maskNet.predict(faces, batch_size=32)

        # return a 2-tuple of the face locations and their corresponding
        # locations
        return (locs, preds)


class FacialNode(Node):
    def __init__(self):
        super().__init__('facial_node')

        self.face_mask_detector = FaceMaskDetector(self.get_logger())

        self.subscription = self.create_subscription(
            RobotImageInfo,
            '/camera/robot_image_info',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(String, '/aai4r/facial', 10)

        self.visualize_flag = True

        self.monitor_publisher = self.create_publisher(Image, '/aai4r/facial/monitor', 1)
        self.cv_bridge = CvBridge()


    def publish_img(self, img):
        self.monitor_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def visualize(self, frame, face_locs, mask_dets):
        for (box, pred) in zip(face_locs, mask_dets):
            cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255,0,0))
        return frame


    def callback(self, msg):
        stamp = msg.stamp
        now = self.get_clock().now().to_msg()
        nano_diff = stamp.nanosec - now.nanosec
        if abs(nano_diff) > 50000000:
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #frame = imutils.resize(im, width=400)

        face_locs, mask_dets = self.face_mask_detector.detect_and_predict_mask(
            frame)

        msg_data = {"timestamp":(stamp.sec,stamp.nanosec), "facial": []}
        for (box, pred) in zip(face_locs, mask_dets):
            # print("FACE: ", box, " / MASK: ",
            #      "ON" if pred[0] > pred[1] else "OFF")
            msg_data["facial"].append(
                {"box": (int(box[0]), int(box[1]), int(box[2]), int(box[3])),
                "mask": bool(pred[0] > pred[1])})
        #self.get_logger().info(json.dumps(msg_data))
        pub_msg = String()
        pub_msg.data = json.dumps(msg_data)
        self.publisher_.publish(pub_msg)

        if self.visualize_flag:
            self.publish_img(self.visualize(frame, face_locs, mask_dets))


def main(args=None):
    rclpy.init(args=args)

    node = FacialNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
