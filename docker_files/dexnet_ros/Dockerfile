#FROM dexnetfull
FROM dexnet-base

# ROS installation

# install packages
RUN apt-get update && apt-get install -q -y \
    vim \
    dirmngr \
    gnupg2 \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core \
    ros-kinetic-ros-base \
    ros-kinetic-robot \
    ros-kinetic-perception \
    ros-kinetic-desktop \
    ros-kinetic-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# setup entrypoint
#COPY ./ros_entrypoint.sh /
COPY ./56-orbbec.rules /etc/udev/rules.d/

#ENTRYPOINT ["/ros_entrypoint.sh"]
#CMD ["bash"]




WORKDIR /app/deps/gqcnn/
# Replace 1000 with your user / group id
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

#setup of ROS packages for astra camero

RUN apt-get update && apt-get install -y git build-essential python libusb-1.0-0-dev freeglut3-dev openjdk-8-jdk vim less ranger

RUN apt-get install -y doxygen graphviz mono-complete ros-kinetic-rgbd-launch

#RUN service udev reload
#RUN service udev restart
ENV HOME /home/developer
WORKDIR /home/developer
USER developer

RUN sudo git clone https://github.com/occipital/OpenNI2.git

RUN sudo apt-get install -y python-catkin-tools
WORKDIR /home/developer/OpenNI2/
RUN sudo make
RUN sudo sh ./Packaging/Linux/install.sh

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
RUN sudo /bin/bash -c "source ~/.bashrc"
#RUN sudo /bin/bash -c "source /opt/ros/kinetic/setup.bash"
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash"
RUN mkdir -p /home/developer/catkin_ws/src
RUN mkdir -p /home/developer/catkin_ws/devel
RUN sudo sh -c su - developer "catkin init"
WORKDIR /home/developer/catkin_ws/src
RUN sudo git clone https://github.com/orbbec/ros_astra_camera.git
RUN sudo git clone https://github.com/orbbec/ros_astra_launch
RUN sudo git clone https://github.com/BerkeleyAutomation/autolab_core.git
RUN sudo git clone https://github.com/BerkeleyAutomation/perception
RUN sudo git clone https://github.com/BerkeleyAutomation/gqcnn.git

WORKDIR /home/developer/catkin_ws/src/autolab_core
RUN sudo python setup.py install

RUN sudo pip install numpy==1.11.1

WORKDIR /home/developer/catkin_ws/src/perception
RUN sudo python setup.py install

#RUN sudo apt-get install -y nvidia-331

WORKDIR /home/developer/
COPY ./catkin-cmds.sh /home/developer/catkin-cmds.sh
RUN sudo chmod +x catkin-cmds.sh
RUN ["/bin/bash", "-c", "./catkin-cmds.sh"]
WORKDIR /home/developer/catkin_ws

USER root

#nvidia gpu stuff
# RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates apt-transport-https gnupg-curl && \
#     rm -rf /var/lib/apt/lists/* && \
#     NVIDIA_GPGKEY_SUM=d1be581509378368edeec8c1eb2958702feedf3bc3d17011adbf24efacce4ab5 && \
#     NVIDIA_GPGKEY_FPR=ae09fe4bbd223a84b2ccfce3f60f4b3d7fa2af80 && \
#     apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub && \
#     apt-key adv --export --no-emit-version -a $NVIDIA_GPGKEY_FPR | tail -n +5 > cudasign.pub && \
#     echo "$NVIDIA_GPGKEY_SUM  cudasign.pub" | sha256sum -c --strict - && rm cudasign.pub && \
#     echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
#     echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1604/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list
#
# ENV CUDA_VERSION 9.0.176
#
# ENV CUDA_PKG_VERSION 9-0=$CUDA_VERSION-1
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         cuda-cudart-$CUDA_PKG_VERSION && \
#     ln -s cuda-9.0 /usr/local/cuda && \
#     rm -rf /var/lib/apt/lists/*
#
# # nvidia-docker 1.0
# LABEL com.nvidia.volumes.needed="nvidia_driver"
# LABEL com.nvidia.cuda.version="${CUDA_VERSION}"
#
# RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf && \
#     echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf
#
# ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
# ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64
#
# # nvidia-container-runtime
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
# ENV NVIDIA_REQUIRE_CUDA "cuda>=9.0"
#
#
# #required for opengl ROS rendering
#
# ENV NCCL_VERSION 2.3.5
#
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         cuda-libraries-$CUDA_PKG_VERSION \
#         cuda-cublas-9-0=9.0.176.4-1 \
#         libnccl2=$NCCL_VERSION-2+cuda9.0 && \
#     apt-mark hold libnccl2 && \
#     rm -rf /var/lib/apt/lists/*
#
# RUN apt-get update && apt-get install -y --no-install-recommends \
#         cuda-libraries-dev-$CUDA_PKG_VERSION \
#         cuda-nvml-dev-$CUDA_PKG_VERSION \
#         cuda-minimal-build-$CUDA_PKG_VERSION \
#         cuda-command-line-tools-$CUDA_PKG_VERSION \
#         cuda-core-9-0=9.0.176.3-1 \
#         cuda-cublas-dev-9-0=9.0.176.4-1 \
#         libnccl-dev=$NCCL_VERSION-2+cuda9.0 && \
#     rm -rf /var/lib/apt/lists/*
#
# ENV LIBRARY_PATH /usr/local/cuda/lib64/stubs

# RUN echo "source source devel/setup.bash" >> ~/.bashrc
# RUN touch ~/.bashrc
# RUN ["/bin/bash", "-c", "source ~/.bashrc"]

#RUN sudo /bin/bash -c "source ./devel/setup.bash"
#RUN roscd astra_camera && ./scripts/create_udev_rules
