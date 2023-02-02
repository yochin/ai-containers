import sys
import json
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from aai4r_edge_interfaces.msg import RobotImageInfo

import cv2
import json

from module_age_estimation import AgeEstimation
from module_face_detection import FaceDetection
from module_gender_classification import GenderClassification
from module_hair_info import HairInfo
from module_mask_detection import MaskDetection
from module_verification_arc import Verification
from module_accessory_detection import AccessoryDetection
from module_emotion_classification import EmotionClassification
import ETRI_Extra_Information as EEI

from facial.visualization import ObjectResult, draw_labeled_boxes

class FacialAttributesDetection(object):
    """
    A wrapper class for facial attributes detection modules.
    Parameters
    ----------
    None
    """

    def __init__(self) -> None:
        super().__init__()



    def load_models(self, path="./models"):
        """
        Load multiple models for facial attributes detection.
        Parameters
        ----------
        path : string
            Path to the folder where models exist.
        Returns
        -------
        x : bool
            True if successful, False otherwise.
        """

        self.module_age = AgeEstimation(path)
        self.module_face = FaceDetection(path)
        self.module_gender = GenderClassification(path)
        self.module_hair = HairInfo(path)
        self.module_mask = MaskDetection(path)
        self.module_fv = Verification(path)
        self.module_ad = AccessoryDetection(path)
        self.module_ec = EmotionClassification(path)
        self.eei = EEI.ETRI_Extra_Information(path)

        return True

    def _inference_module(self, frame, idx):
        # 1. mask detection
        self.module_mask.run(frame, self.last_info, idx)

        # 2. verification & save feature
        aligned = self.module_fv.faceAlignment_arc(frame, self.last_info[idx].ptLE,
                                                   self.last_info[idx].ptRE,
                                                   self.last_info[idx].ptLM,
                                                   self.last_info[idx].ptRM,
                                                   self.last_info[idx].ptN)
        self.module_fv.ETRI_Face_Verification(frame, self.last_info, idx, aligned)

        # 3. make universal aligned image
        aligned = self.module_gender.faceAlignment(frame, self.last_info[idx].ptLE,
                                                   self.last_info[idx].ptRE,
                                                   self.last_info[idx].ptLM,
                                                   self.last_info[idx].ptRM)

        self.module_gender.run(frame, self.last_info, idx, aligned)
        self.module_age.run(frame, self.last_info, idx, aligned)
        self.module_ec.run(frame, self.last_info, idx, aligned)

        # accessory
        self.module_ad.run(frame, self.last_info, idx)

        # other information
        ### Hair Information
        # Hair Length
        self.eei.ETRI_Hair_Length(frame, self.last_info, idx)
        # print("Hair Length : %d" % list_ETRIFace[ii].nHairLength)

        # Hair Brightness
        self.eei.ETRI_Hair_Color(frame, self.last_info, idx)
        # print("Hair Brightness : %d" % list_ETRIFace[ii].nHairBright)

        # Hair Style
        self.eei.ETRI_Hair_Style(frame, self.last_info, idx)
        # print("Hair Style : %d" % list_ETRIFace[ii].nHairStyle)

        # Lip Color
        self.eei.ETRI_LipColor_Classification(frame, self.last_info, idx)
        # print("LipColor : %d" % list_ETRIFace[ii].nLipColor)

        # Nation
        self.eei.ETRI_Nation_Classification(frame, self.last_info, idx)

    def infer(self, img):
        """
        Perform an inference to detect facial attributes and return the results.
        Parameters
        ----------
        img : array_like
            An RGB image as a numpy array shaped in ``(height, width, no of channels)``.
        Returns
        -------
        d : string
            A Json string containing detection results
        """

        self.last_info = []

        idx = self.module_face.run(img, self.last_info)
        for ii in range(len(self.last_info)):
            self._inference_module(img, ii)

        results = self.parse_result(self.last_info)

        return results




    def parse_result(self, list_ETRIFace):
        '''
        [
          # 이미지 안에 존재하는 사람들에 대해 검출 결과를 아래와 같은 구조체의 목록으로 제공
          { # 사람 1
             'identities': [('user012345',0.7545), ('user03421',0.214), ('user04354', 0.03)], # 신원 목록 (아이디, 신뢰도)의 Tuple을 Top-3까지 포함
             'face_bbox': (100,100,200,200),                                                  # 얼굴 영역 Bbox 좌표 (left,top,right,bottom), 얼굴이 없으면 (-1,-1,-1,-1)
             'gender': (0,0.987),                                                             # 성별: 여자(0), 남자(1)와 신뢰도의 쌍을 제공
             'age': (15.3,0.778),                                                             # 연령: 연령 추정값과 신뢰도의 쌍을 제공
             'nationality': 1,                                                                # 국적: 한국(1), 미국(2), 중국(3), 일본(4), 베트남(5), 이란(6),
                                                                                              #      인도(7), 필리핀(8), 멕시코(9), 독일(10), 인도네시아(11),
                                                                                              #      태국(12), 호주(13), 사우디아라비아(14), 러시아(15)
             'skin_color': 0,                                                                 # 피부색: 황(0), 흑(1), 백(2)
             'expression': [(0,0.5), (1,0.2), (2,0.2), (5,0.1)],                              # 표정: (표정, 신뢰도)의 Tuple 목록 (중립(0), 기쁨(1), 화남(2), 놀람(3), 슬픔(4))
             'hair_style': 1,                                                                 # 머리 스타일: 직모(0), 곱슬(1), 반곱슬(2)
             'hair_color': 1,                                                                 # 머리 색상: 검정색(0), 흰색(1), 갈색(2)
             'hair_length': 0,                                                                # 머리 길이: 짧음(0), 길음(1), 중간(2)
             'earring': [(100,100,200,200),(300,300,400,400)],                                # 귀걸이 영역 Bbox 목록 (left,top,right,bottom)
             'necklace': [(100,100,200,200),(300,300,400,400)],                               # 목걸이 영역 Bbox 목록 (left,top,right,bottom)
             'mask': (100,100,200,200),                                                       # 마스크 영역 Bbox 좌표 (left,top,right,bottom), 마스크 없으면 (-1,-1,-1,-1)
             'lip_color': 0,                                                                  # 입술 색상: 갈색(0), 빨강색(1), 분홍색(2)
          },
          { # 사람 2
            ...
          },
          {
            ...
          }
        ]
        '''
        results = []
        for ii in list_ETRIFace:
            results.append({ # 사람 1
                "identities": [ii.sID, (float)(ii.fvScore)],                                             # 일단 Top1의 정보만 제공
                "face_bbox": list(ii.rt),                                                                # 얼굴 영역 Bbox 좌표 (left,top,right,bottom), 얼굴이 없으면 (-1,-1,-1,-1)
                "gender": [ii.fGender, [(float)(ii.fGenderProb[0]), (float)(ii.fGenderProb[1])]],        # 성별: 여자(0), 남자(1)와 신뢰도의 쌍을 제공
                "age": (float)(ii.fAge),                                                                 # 연령: 연령 추정값을 제공
                "expression": [[0, (float)(ii.fEmotionScore[6])],
                               [1, (float)(ii.fEmotionScore[3])],
                               [2, (float)(ii.fEmotionScore[5])],
                               [3, (float)(ii.fEmotionScore[0])],
                               [4, (float)(ii.fEmotionScore[4])]],                              # 표정: (표정, 신뢰도)의 Tuple 목록 (중립(0), 기쁨(1), 화남(2), 놀람(3), 슬픔(4))
                "mask": [ii.fMask.item(), (float)(ii.fMaskScore)],                              # 마스크 착용여부와 신뢰도의 쌍으로 제공
                "hair_style": ii.nHairStyle,                                                    # 머리 스타일: ["Straight", "Wavy", "Hair detect fail"]
                "hair_color": ii.nHairColor,                                                    # 머리 색상: ["Dark", "Light", "Ambiguous"]
                "hair_length": ii.nHairLength,                                                  # 머리 길이: ["Short", "Normal", "Long", "Hair detect fail"]

                "lip_color": ii.nLipColor,                                                      # 입술 색상: 갈색(0), 빨강색(1), 분홍색(2)

                "nationality": ii.nNation,                                                      # 국적: ["중국", "인도네시아", "일본", "사우디아라비아", "독일",
                                                                                                # "베트남", "미국", "이란", "호주", "태국",
                                                                                                # "러시아", "멕시코", "인도", "한국", "필리핀"]

                "necklace": ii.nNecklacePos,                                                    # 목걸이 영역 Bbox 목록 (left,top,right,bottom), 없으면 []
                "earringL": ii.nEarringLPos,                                                    # 큰귀걸이 영역 Bbox 목록 (left,top,right,bottom), 없으면 []
                "earringS": ii.nEarringSPos,                                                    # 작은귀걸이 영역 Bbox 목록 (left,top,right,bottom), 없으면 []

                # 아래 기능은 추가 구현 필요.
                "skin_color": 0,                                                                # 피부색: 황(0), 흑(1), 백(2)
          })

        return results


class FacialNode(Node):
    def __init__(self):
        super().__init__('facial_node')

        self.get_logger().info("facial_node initialization begin...")

        self.get_logger().info("   creating a deep learning-based facial attribute detector...")
        self.facial_attribute_detector = FacialAttributesDetection()
        self.get_logger().info("   created a deep learning-based facial attribute detector...")
        self.facial_attribute_detector.load_models("./facial-ros2/models")
        self.get_logger().info("   loaded models for the facial attribute detector...")

        self.subscription = self.create_subscription(
            RobotImageInfo,
            '/camera/robot_image_info',
            self.callback,
            1)
        self.subscription  # prevent unused variable warning

        self.get_logger().info("   subscribed to /camera/robot_image_info...")

        self.publisher_ = self.create_publisher(String, '/aai4r/facial', 10)

        self.get_logger().info("   a publisher created for /aai4r/facial...")

        self.visualize_flag = True

        self.monitor_publisher = self.create_publisher(Image, '/aai4r/facial/monitor', 1)
        self.cv_bridge = CvBridge()

        self.get_logger().info("facial node initialized!")


    def publish_img(self, img):
        self.monitor_publisher.publish(self.cv_bridge.cv2_to_imgmsg(img, "bgr8"))


    def visualize(self, frame, facial_attrs):
        results = []
        count = 0
        for attr in facial_attrs:
            box = attr['face_bbox']
            age = int(attr['age'])
            gender = 'M' if attr['gender'][0] == 1 else 'F'
            mask = 'NO_MASK' if attr['mask'][0] == 0 else 'MASK_ON'
            results.append(ObjectResult(count, '{}/{}/{}'.format(gender, age, mask), 0.9, box))
            count = count + 1
            #cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (255,0,0))
        frame = draw_labeled_boxes(frame, results)
        return frame


    def callback(self, msg):
        self.get_logger().info("callback called!")
        stamp = msg.stamp
        now = self.get_clock().now().to_msg()
        nano_diff = stamp.nanosec - now.nanosec
        if abs(nano_diff) > 50000000:
            return
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        #frame = imutils.resize(im, width=400)

        results = self.facial_attribute_detector.infer(frame)

        #self.get_logger().info(json.dumps(msg_data))
        msg_data = {"timestamp":(stamp.sec,stamp.nanosec), "agent_id":msg.agent_id, "facial": []}
        msg_data["facial"] = results
        pub_msg = String()
        pub_msg.data = json.dumps(msg_data)
        
        self.get_logger().info(pub_msg.data)
        
        self.publisher_.publish(pub_msg)

        if self.visualize_flag:
            self.publish_img(self.visualize(frame, results))


def main(args=None):
    rclpy.init(args=args)

    node = FacialNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
