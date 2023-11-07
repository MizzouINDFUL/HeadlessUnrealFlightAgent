FROM ros:noetic-ros-core

WORKDIR /

RUN apt update && apt install -y ros-noetic-rosbridge-suite

RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    libboost-iostreams-dev \
    libboost-numpy-dev \
    libboost-python-dev \
    libtbb-dev \
    libblosc-dev \
    mesa-utils

RUN pip3 install rospkg

RUN sudo apt install -y python3-numpy &&\
    sudo apt install -y libboost-python-dev

RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y

RUN pip install cvbridge3

RUN pip3 install opencv-python && \
	pip3 install msgpack-rpc-python

COPY init-rosbridge.sh .

RUN chmod +x init-rosbridge.sh

CMD ["/init-rosbridge.sh"]
