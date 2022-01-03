FROM nvidia/cuda:11.0-devel-ubuntu20.04 as torch_base
# ubuntu version 20.04

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y software-properties-common
RUN apt-get update && add-apt-repository universe
RUN apt-get update && apt-get install -y curl gnupg lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

ENV ROS_DISTRO foxy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-core=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-base=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-desktop=0.9.2-1* \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

############################### TORCH
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install torch==1.8.1+cu111 torchvision==0.9.1+cu111 torchaudio==0.8.1 -f https://download.pytorch.org/whl/torch_stable.html

############################### INTELLIGENCE MODULE
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
RUN apt-get update && apt-get install -y tzdata

RUN pip install pretrainedmodels
RUN pip install numpy
RUN pip install imageio
RUN pip install easydict

RUN apt-get update && apt-get install -y python3-opencv
RUN pip install opencv-python

RUN pip install imutils
RUN sudo apt-get update
RUN sudo apt-get install fonts-nanum

RUN mkdir /aai4r

WORKDIR /aai4r
RUN git clone https://github.com/aai4r/aai4r-ServiceContextUnderstanding.git

RUN export PYTHONPATH=$PYTHONPATH:/aai4r/aai4r-ServiceContextUnderstanding
RUN export PYTHONPATH=$PYTHONPATH:/aai4r/aai4r-ServiceContextUnderstanding/lib

WORKDIR /aai4r/aai4r-ServiceContextUnderstanding
RUN mkdir output

COPY ./gdrivedl.py /aai4r/aai4r-ServiceContextUnderstanding/output
COPY ./download_models.sh /aai4r/aai4r-ServiceContextUnderstanding/output
COPY ./download_senet.py /aai4r/aai4r-ServiceContextUnderstanding/output
COPY ./download_senet.sh /aai4r/aai4r-ServiceContextUnderstanding/output

WORKDIR /aai4r/aai4r-ServiceContextUnderstanding/output
RUN /aai4r/aai4r-ServiceContextUnderstanding/output/download_models.sh

COPY ./food_detector_interface.py /aai4r/aai4r-ServiceContextUnderstanding

RUN mkdir -p /root/.cache/torch/hub/checkpoints
RUN /aai4r/aai4r-ServiceContextUnderstanding/output/download_senet.sh

COPY ./aai4r_edge_interfaces/ /aai4r/aai4r_edge_interfaces/
COPY ./meal-ros2 /aai4r/meal-ros2/

WORKDIR /aai4r
RUN sudo rm -rf install build log

COPY ./run_colcon.sh /aai4r
RUN /aai4r/run_colcon.sh

COPY ./run_meal.sh /aai4r
COPY ./ros_entrypoint.sh /aai4r

ENTRYPOINT ["/aai4r/ros_entrypoint.sh"]
CMD ["/aai4r/run_meal.sh"]

