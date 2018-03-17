FROM ubuntu:latest
LABEL maintainer="keybase.io/vwiart"

RUN mkdir -p /etc/ros/rosdep/sources.list.d/ && \
    echo "deb [ arch=amd64 ] http://packages.dataspeedinc.com/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-dataspeed-public.list && \
    echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 66F84AE1EB71A8AC108087DCAF677210FF6D3CDA && \
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    echo "yaml http://packages.dataspeedinc.com/ros/ros-public-kinetic.yaml kinetic" > /etc/ros/rosdep/sources.list.d/30-dataspeed-public-kinetic.list && \
    apt-get update && \
    apt-get install -y curl git libuv1-dev libssl-dev gcc g++ cmake make zlib1g-dev ros-kinetic-desktop-full python-pip \
        ros-kinetic-dbw-mkz ros-kinetic-cv-bridge ros-kinetic-pcl-ros ros-kinetic-image-proc netbase

ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/opt/ros/kinetic/bin
COPY requirements.txt requirements.txt
RUN pip install --upgrade pip && \
    pip install -r requirements.txt && rm requirements.txt

RUN echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc

WORKDIR /app
RUN mkdir -p /app
