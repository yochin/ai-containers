# detrack

## Introduction

``detrack`` detects and tracks people in a video input, which is implemented as a docker container with a ROS2 node and core AI components as follows:

- detrack core module: for human detection and tracking
- detrack ROS2 node: for executing the detrack as a ROS2 node
- detrack containerization scripts: for creating a docker container for ```detrack```

## Folders and Files

- ``detrack-ros2``: An implementation of a ROS2 node for ``detrack``
- ``detrack_aai4r.Dockerfile``: A dockerfile for creating a ``detrack`` container image
- ``dockerbuild_detracker.sh``: A shell script for creating a container image with the ``detrack_aai4r.Dockerfile`` and tagging it properly
- ``ros_entrypoint.sh``: An entrypoint shell script for the container
- ``run_colcon.sh``: A shell script to build the ROS2 node inside the container
- ``run_container_detrack.sh``: A shell script to create a container and execute
- ``run_detrack.sh``: A shell script to run the ROS2 node inside the container
- ``run_model_download.sh``: A shell script to download deep learning models for the ``detrack`` core components, which is run inside the container

## Instructions

To create a docker container, run the following command.

```
dockerbuild_detrack.sh
```