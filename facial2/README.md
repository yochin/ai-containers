# facial

## Introduction

``facial`` detects faces and recognizes facial attributes from each facial region. The attributes include *gender, age, ethinicity, accessories, hair style/color, lip color etc.*

This container interfaces with clients via ROS2 topics.

## Folders and Files

- ``facial-ros2``: An implementation of a ROS2 node for ``facial``
- ``facial_aai4r.Dockerfile``: A dockerfile for creating a ``facial`` container image
- ``requirements.txt``: A list of required packages to be installed inside the image
- ``dockerbuild_facial.sh``: A shell script for creating a container image with the ``facial_aai4r.Dockerfile`` and tagging it properly
- ``ros_entrypoint.sh``: An entrypoint shell script for the container
- ``run_colcon.sh``: A shell script to build the ROS2 node inside the container
- ``run_container_facial.sh``: A shell script to create a container and execute
- ``run_facial.sh``: A shell script to run the ROS2 node inside the container
- ``run_model_download.sh``: A shell script to download deep learning models for the ``facial`` core components, which is run inside the container

## Instructions

To create a docker container:

```
dockerbuild_facial.sh
```

To create and run a ``facial`` container:

```
run_container_facial.sh
```

## ROS2 Interface

### Subscribed Topics

#### ``/camera/robot_image_info`` ([RobotImageInfo](https://github.com/aai4r/ai-containers/blob/main/aai4r_edge_interfaces/msg/RobotImageInfo.msg))

### Published Topics

#### ``/aai4r/facial`` ([std_msgs/String](https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/String.msg))

This topic publishes a string which contains all the detection and recognition results. The string is formatted in ``JSON`` and the content is structured as the following:

```json
[
  {
    // Identity recognition
    "identities": [<ID>, <SCORE>],                               
    // BBox (left,top,right,bottom)
    "face_bbox": [<LEFT>, <TOP>, <RIGHT>, <BOTTOM>],                                         
    // Gender: <GENDER_ID> = {Female(0), Male(1)}
    "gender": [<GENDER_ID>, <SCORE>],                             
    // Age: floating number representing a predicted age
    "age": <AGE_VALUE>,
    // Facial expression recognition: <EXPRESSION_ID> = {Neutral(0), Happy(1), Angry(2), Surprise(3), Sad(4))
    "expression": [[<EXPRESSION_ID>, <SCORE>],...],
    // Mask detection: <MASK> = {0: OFF, 1: ON}
    "mask": [<MASK>, <SCORE>],

    // Hair Style detection: <HAIR_STYLE> = ["Straight", "Wavy", "Hair detect fail"]
    "hair_style": <HAIR_STYLE>,
    // Hair Color detection: <HAIR_COLOR> = ["Dark", "Light", "Ambiguous"]
    "hair_color": <HAIR_COLOR>,
    // Hair Length detection: <HAIR_LENGTH> = ["Short", "Normal", "Long", "Hair detect fail"]
    "hair_length": <HAIR_LENGTH>,

    // Lip Color: <LIP_COLOR> = {Brown(0), Red(1), Pink(2)}
    "lip_color": <LIP_COLOR>,
    // Necklace Bbox list (left,top,right,bottom)
    "necklace": [<LEFT>, <TOP>, <RIGHT>, <BOTTOM>],
    // Nationality: <COUNTRY_CODE> = ["중국", "인도네시아", "일본", "사우디아라비아", "독일",
    // "베트남", "미국", "이란", "호주", "태국",
    // "러시아", "멕시코", "인도", "한국", "필리핀"]
    "nationality": <COUNTRY_CODE>,

    // Earring BBox list (left,top,right,bottom)
    "earring": [[<LEFT>, <TOP>, <RIGHT>, <BOTTOM>],[<LEFT>, <TOP>, <RIGHT>, <BOTTOM>], ...],
    // Skin Color detection: <COLOR_ID> = {Yellow(0), Black(1), White(2)}
    "skin_color": <COLOR_ID>,
  },
  ...
]

```

#### ``/aai4r/facial/monitor`` ([sensor_msgs/Image](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Image.msg))


This topic publishes an image containing visualizations of the detections and recognitions predicted by ``facial``.

