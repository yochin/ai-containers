FROM ubuntu:focal

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

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-image-transport-plugins

############################### INTELLIGENCE MODULE
RUN apt-get update && apt-get install -y python3-pip
RUN apt-get update && apt-get install -y python3-opencv
RUN pip3 install opencv-python

RUN pip3 install Pillow
RUN apt-get update && apt-get install -y ttf-ubuntu-font-family

RUN apt-get update && apt-get install -y x264

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
RUN apt-get update && apt-get install -y tzdata

RUN mkdir /aai4r

COPY ./aai4r_edge_interfaces/ /aai4r/aai4r_edge_interfaces/
COPY ./loom_ros2 /aai4r/loom_ros2/
WORKDIR /aai4r
RUN sudo rm -rf install build log

COPY ./run_colcon.sh /aai4r
RUN /aai4r/run_colcon.sh

COPY ./run_loom.sh /aai4r
COPY ./ros_entrypoint.sh /aai4r

RUN mkdir /video_backup

ENTRYPOINT ["/aai4r/ros_entrypoint.sh"]
CMD ["/aai4r/run_loom.sh"]

