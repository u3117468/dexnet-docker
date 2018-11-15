#FROM ubuntu:16.04
FROM nvidia/cudagl:9.0-devel-ubuntu16.04
WORKDIR /app
ADD . /app
RUN apt-get update && apt-get upgrade -y
RUN apt-get install --no-install-recommends -y python-pip git cmake make sudo gcc libboost-all-dev build-essential python2.7 python-dev assimp-utils libassimp-dev libgtk2.0-dev python-vtk python-tk
RUN pip install --upgrade pip
RUN pip install --upgrade setuptools
RUN /app/dex-net/install.sh cpu python
WORKDIR /app/dex-net/
RUN sh install_3rdparty_deps.sh
RUN python setup.py develop

ENV DEBIAN_FRONTEND noninteractive
RUN apt-get install -y -qq keyboard-configuration
RUN apt-get install -y meshlab freeglut3-dev libxmu-dev libxi-dev libosmesa6-dev libgl1-mesa-glx
RUN pip2 install --upgrade tensorflow==1 numpy==1.11 IPython==5.5

WORKDIR /app/dex-net/
RUN mkdir ./home/
ENV HOME=/apps/home/

RUN echo "/usr/local/lib" >> /etc/ld.so.conf
RUN echo "/usr/local/lib64" >> /etc/ld.so.conf
RUN ldconfig

WORKDIR /app
ADD . /app
WORKDIR /app/dex-net/
