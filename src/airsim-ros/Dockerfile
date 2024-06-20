FROM ros:noetic-ros-core

#Install ROS
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

# Create a setup script
ADD setup.sh /root/setup.sh

# Set up .bashrc
RUN cp /etc/skel/.bashrc /root/.bashrc
RUN echo "source /root/setup.sh" >> /root/.bashrc

COPY init-rosbridge.sh .

RUN chmod +x init-rosbridge.sh

# Install AirSim
RUN apt-get update && apt-get install -y git ros-noetic-catkin ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-mavros*
RUN . /opt/ros/noetic/setup.sh && \
    git clone https://github.com/Microsoft/AirSim.git && \
    cd AirSim && \
    ./setup.sh && \
    ./build.sh && \
    # cd ros && \
    # catkin build && \
	pip install opencv-python && \
	pip install msgpack-rpc-python && \
	pip install airsim && \
	pip install cvbridge3 && \
    pip install pyyaml

#Install OpenEXR for Python
RUN apt-get install -y libopenexr-dev && \
    pip install openexr