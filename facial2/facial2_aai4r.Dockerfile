FROM nvidia/cuda:11.4.3-cudnn8-devel-ubuntu20.04 as torch_base
# ubuntu version 20.04

#RUN apt-get update && apt-get upgrade
#RUN apt-key del 7fa2af80
#RUN curl -L -O https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-keyring_1.0-1_all.deb
#RUN dpkg -i cuda-keyring_1.0-1_all.deb

RUN rm /etc/apt/sources.list.d/cuda.list
#RUN rm /etc/apt/sources.list.d/nvidia-ml.list
RUN apt-key del 7fa2af80
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64/7fa2af80.pub

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
#RUN pip3 install torch==1.8.1+cu111 torchvision==0.9.1+cu111 torchaudio==0.8.1 -f https://download.pytorch.org/whl/torch_stable.html

############################### INTELLIGENCE MODULE
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul
RUN apt-get update && apt-get install -y tzdata

RUN mkdir /aai4r
WORKDIR /aai4r
RUN git clone https://oauth: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@github.com/zebehn/aai4r-facial.git
RUN export PYTHONPATH=$PYTHONPATH:/aai4r/aai4r-facial

WORKDIR /aai4r/aai4r-facial
RUN pip install -r /aai4r/aai4r-facial/requirements.txt
RUN pip uninstall torch -y
RUN pip install torch==1.9.0+cu111 torchvision==0.10.0+cu111 torchaudio==0.9.0 -f https://download.pytorch.org/whl/torch_stable.html
RUN pip3 install Pillow
RUN apt-get update && apt-get install -y ttf-ubuntu-font-family
RUN apt-get update && apt-get install -y x264

COPY ./aai4r_edge_interfaces/ /aai4r/aai4r_edge_interfaces/
COPY ./facial-ros2 /aai4r/facial-ros2/
WORKDIR /aai4r/facial-ros2/models
#RUN /aai4r/facial-ros2/models/model_download.sh

WORKDIR /aai4r
RUN sudo rm -rf install build log

COPY ./run_colcon.sh /aai4r
RUN /aai4r/run_colcon.sh

COPY ./run.sh /aai4r
COPY ./ros_entrypoint.sh /aai4r

ENTRYPOINT ["/aai4r/ros_entrypoint.sh"]
CMD ["/aai4r/run.sh"]

