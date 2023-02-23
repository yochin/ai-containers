# dummybot

## Introduction

A mock robot container for transmitting a series of images captured from a webcam.

## Folders and Files

- ``aai4r_camera``: An implementation of a ROS2 node for publising images in ``/aai4r_edge_interfaces/RobotImageInfo.msg``
- ``Dockerfile``: A dockerfile for creating a ``dummybot`` container image
- ``dockerbuild.sh``: A shell script for creating a container image with the ``Dockerfile`` and tagging it properly
- ``ros_entrypoint.sh``: An entrypoint shell script for the container
- ``run_colcon.sh``: A shell script to build the ROS2 node inside the container
- ``run_container.sh``: A shell script to create a container and execute
- ``run.sh``: A shell script to run the ROS2 node inside the container

## Instructions

To create a docker container:

```
dockerbuild.sh
```

To run a ``dummybot`` container:

```
run_container.sh
```

To check if the image is published:

```
run_container_showimg.sh
```

## ROS2 Interface

### Published Topics

#### ``/camera/robot_image_info`` ([RobotImageInfo](https://github.com/aai4r/ai-containers/blob/main/aai4r_edge_interfaces/msg/RobotImageInfo.msg))

