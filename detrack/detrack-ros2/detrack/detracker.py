import os
import sys
import json
from typing import Sequence
from urllib.request import urlretrieve

import cv2
from motpy import Detection, MultiObjectTracker, NpImage, Box
from motpy.core import setup_logger
from motpy.detector import BaseObjectDetector
from motpy.testing_viz import draw_detection, draw_track

import torch

#from deepface import DeepFace

logger = setup_logger(__name__, 'INFO', is_main=True)


WEIGHTS_URL = 'https://github.com/opencv/opencv_3rdparty/raw/dnn_samples_face_detector_20170830/res10_300x300_ssd_iter_140000.caffemodel'
WEIGHTS_PATH = 'opencv_face_detector.caffemodel'
CONFIG_URL = 'https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/face_detector/deploy.prototxt'
CONFIG_PATH = 'deploy.prototxt'


class ObjectDetector(object):
    def __init__(self):
        super(ObjectDetector, self).__init__()
        # Model
        repo = 'ultralytics/yolov5'
        self.model = torch.hub.load(repo, 'yolov5m', pretrained=True)
        self.conf_threshold: float = 0.5

    # class id
    # 0: person
    def process_image(self, image):
        results = self.model(image)
        results = results.pandas().xyxy[0].to_numpy()

        # convert output from OpenCV detector to tracker expected format [xmin, ymin, xmax, ymax]
        out_detections = []
        for i in range(results.shape[0]):
            # filter out non-human objects
            if results[i,5] != 0:
                continue
            confidence = results[i, 4]
            if confidence > self.conf_threshold:
                xmin = int(results[i, 0])
                ymin = int(results[i, 1])
                xmax = int(results[i, 2])
                ymax = int(results[i, 3])
                out_detections.append(Detection(box=[xmin, ymin, xmax, ymax], score=confidence, feature=results[i,5]))

        return out_detections


class FaceDetector(BaseObjectDetector):
    def __init__(self,
                 weights_url: str = WEIGHTS_URL,
                 weights_path: str = WEIGHTS_PATH,
                 config_url: str = CONFIG_URL,
                 config_path: str = CONFIG_PATH,
                 conf_threshold: float = 0.5) -> None:
        super(FaceDetector, self).__init__()

        if not os.path.isfile(weights_path) or not os.path.isfile(config_path):
            logger.debug('downloading model...')
            urlretrieve(weights_url, weights_path)
            urlretrieve(config_url, config_path)

        self.net = cv2.dnn.readNetFromCaffe(config_path, weights_path)
        #faces = DeepFace.detectFace('test_img.jpg')
        #print(faces)

        # specify detector hparams
        self.conf_threshold = conf_threshold

    def process_image(self, image: NpImage) -> Sequence[Detection]:
        # DeepFace is too slow
        #faces = DeepFace.detectFace(image)
        #print(faces)

        blob = cv2.dnn.blobFromImage(image, 1.0, (300, 300), [104, 117, 123], False, False)
        self.net.setInput(blob)
        detections = self.net.forward()
        # convert output from OpenCV detector to tracker expected format [xmin, ymin, xmax, ymax]
        out_detections = []
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > self.conf_threshold:
                xmin = int(detections[0, 0, i, 3] * image.shape[1])
                ymin = int(detections[0, 0, i, 4] * image.shape[0])
                xmax = int(detections[0, 0, i, 5] * image.shape[1])
                ymax = int(detections[0, 0, i, 6] * image.shape[0])
                out_detections.append(Detection(box=[xmin, ymin, xmax, ymax], score=confidence, feature=555))
        return out_detections


class Detracker(object):
    def __init__(self, display=False):
        super(Detracker, self).__init__()

        self.display = display

        # prepare multi object tracker
        self.model_spec = {'order_pos': 1, 'dim_pos': 2,
                    'order_size': 0, 'dim_size': 2,
                    'q_var_pos': 5000., 'r_var_pos': 0.1}

        self.tracker = MultiObjectTracker(dt=1/20.0, model_spec=self.model_spec)

        self.object_detector = ObjectDetector()
        #self.face_detector = FaceDetector()

    def process_frame(self, frame):
        # run face detector on current frame
        detections = self.object_detector.process_image(frame)
        logger.debug(f'detections: {detections}')

        self.tracker.step(detections)
        tracks = self.tracker.active_tracks(min_steps_alive=3)
        logger.debug(f'tracks: {tracks}')

        if self.display:
            for track in tracks:
                draw_track(frame, track)

            # preview the boxes on frame
            for det in detections:
                if det.feature == 0:
                    roi = frame[det.box[1]:det.box[3],det.box[0]:det.box[2]]
                    #face_dets = self.face_detector.process_image(roi)
                    #if len(face_dets) == 1:
                    #    face_det = face_dets[0]
                    #    cv2.imshow('face', roi[face_det.box[1]:face_det.box[3],face_det.box[0]:face_det.box[2]])
                draw_detection(frame, det)

        return tracks, detections, frame

def run(video_source):
    if len(video) == 1:
        video_source = int(video)

    detracker = Detracker(display=False)

    dt = 1 / 50.0  # assume 15 fps

    # open camera
    cap = cv2.VideoCapture(video_source)
    frame_no = 0
    while True:
        ret, frame = cap.read()

        frame_no = frame_no + 1
        
        if not ret:
            break

        #frame = cv2.resize(frame, dsize=None, fx=0.5, fy=0.5)

        tracks, detections, frame = detracker.process_frame(frame)

        if len(tracks) > 0:
            cv2.imwrite('frames/frame_{:06d}.jpg'.format(frame_no), frame)

        trks = []
        for trk in tracks:
            print(trk)
            trks.append({'track_id':trk.id, 'pos':list(trk.box), 'score':trk.score})
            # cv2.rectangle(frame, (trk.box[0],trk.box[1]), (trk.box[2],trk.box[3]), (255,0,0), 2)
            cv2.rectangle(frame, (int(trk.box[0]), int(trk.box[1])), (int(trk.box[2]), int(trk.box[3])), (255,255,0), 3)

        print(json.dumps(trks))
        print('\n\n')

        cv2.imshow('frame', frame)

        # stop demo by pressing 'q'
        if cv2.waitKey(int(1000 * dt)) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    #test_yolo5()
    if len(sys.argv) < 2:
        print("[USAGE] python detracker <video_name>")
        exit(-1)
    else:
        video = sys.argv[1]
        run(video)
