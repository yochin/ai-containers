FROM nvidia/cuda:11.0-devel-ubuntu20.04 as torch_base
# ubuntu version 20.04

ENV DEBIAN_FRONTEND=noninteractive
ARG PYTHON_VERSION=3.8.5
ARG WITH_TORCHVISION=1

RUN apt-get update && apt-get install -y --no-install-recommends \
   curl \
   ca-certificates \
   sudo \
   git \
   bzip2 \
   libx11-6 \
   && rm -rf /var/lib/apt/lists/*

RUN curl -o ~/miniconda.sh https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh && \
   chmod +x ~/miniconda.sh && \
   ~/miniconda.sh -b -p /opt/conda && \
   rm ~/miniconda.sh && \
   /opt/conda/bin/conda install -y python=$PYTHON_VERSION numpy pyyaml scipy ipython mkl mkl-include ninja cython typing && \
   /opt/conda/bin/conda install -y -c pytorch magma-cuda100 && \
   /opt/conda/bin/conda clean -ya

RUN /opt/conda/bin/conda install -y -c pytorch \
   cudatoolkit=11.0.221 \
   "pytorch=1.7.0=py3.8_cuda11.0.221_cudnn8.0.3_0" \
   "torchvision=0.8.1=py38_cu110" \
   && /opt/conda/bin/conda clean -ya

ENV PATH /opt/conda/bin:$PATH
# This must be done before pip so that requirements.txt is available
WORKDIR /opt/pytorch


FROM torch_base as ros_base

RUN apt-get update -qq \
   && apt-get -y install locales \
   && locale-gen en_US en_US.UTF-8 \
   && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
   && export LANG=en_US.UTF-8

RUN apt-get update -qq \
   && DEBIAN_FRONTEND=noninteractive apt-get -y install curl gnupg2 lsb-release tzdata htop tmux

RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
   && echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list \
   && apt-get update -qq \
   && apt-get -y install --no-install-recommends ros-rolling-ros-base \
   && apt-get autoclean && apt-get clean && apt-get -y autoremove \
   && rm -rf /var/lib/apt/lists/*


# for desktop

RUN apt-get update -qq \
   && apt-get -y install --no-install-recommends ros-rolling-desktop \
   && apt-get autoclean && apt-get clean && apt-get -y autoremove \
   && rm -rf /var/lib/apt/lists/*

RUN \
    groupadd user \ 
    && useradd -rm -d /home/user -s /bin/bash -g user -G sudo user \
    && echo "user:user" | /usr/sbin/chpasswd \
    && echo "user ALL=NOPASSWD: ALL" >> /etc/sudoers


FROM ros_base as vnc_base

USER root

RUN apt-get update && apt-get install -y --no-install-recommends \
   sudo \
   dbus-x11 \
   xvfb \
   x11vnc \
   xfce4 \
   xfce4-terminal \
   x11-xserver-utils \
   at-spi2-core \
   && apt-get autoclean -y \
   && apt-get autoremove -y \
   && rm -rf /var/lib/apt/lists/*


ENV NOVNC_HOME=/noVNC

RUN locale-gen en_US.UTF-8

# install novnc
RUN git clone https://github.com/novnc/noVNC  $NOVNC_HOME \
    && git clone https://github.com/novnc/websockify  $NOVNC_HOME/utils/websockify \
    && chmod +x -v $NOVNC_HOME/utils/*.sh \
    && ln -s $NOVNC_HOME/vnc.html $NOVNC_HOME/index.html \
    && apt-get autoclean \
    && apt-get -y autoremove \
    && rm -rf /var/lib/apt/lists/*


ENV DISPLAY=:1 \
    VNC_RESOLUTION=1600x1200  \
    VNC_COL_DEPTH=24 \
    VNC_PORT=5900 \
    NOVNC_PORT=6080 \
    XFCE_PANEL_MIGRATE_DEFAULT=1

EXPOSE 6080

ENV \
    USER=user \
    HOME=/home/user 


ENV FORCE_CUDA="1"
ENV CUDA_HOME=/usr/local/cuda
RUN pip install --no-cache-dir flask

RUN /opt/conda/bin/conda install -y -c conda-forge \
   rosdep  \
   argcomplete \
   colcon-common-extensions \
   && /opt/conda/bin/conda clean -ya

#RUN apt-get update && \
#    apt-get install -y python-rosdep \
#    libpython3-dev \
#    python3-argcomplete \
#    python3-colcon-common-extensions \
#    && update-alternatives --install /usr/bin/python3 python3 /opt/conda/bin/python3 1

RUN rosdep init

USER user

RUN rosdep fix-permissions \
   && rosdep update  \
   && echo "source /opt/ros/rolling/setup.bash" >> ~/.bashrc

RUN pip install pretrainedmodels
RUN pip install opencv-python
RUN pip install numpy
RUN pip install imageio
RUN pip install easydict

RUN mkdir -p /home/user/workspace

COPY init_vnc.sh /home/user

COPY ./aai4r-ServiceContextUnderstanding/ /home/user/aai4r-ServiceContextUnderstanding/

WORKDIR /home/user/aai4r-ServiceContextUnderstanding
RUN python download_senet.py

ENTRYPOINT bash
