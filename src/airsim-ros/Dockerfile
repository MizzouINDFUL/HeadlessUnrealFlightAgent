FROM mindful-ros-noetic

# Install AirSim
RUN apt-get update && apt-get install -y ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-mavros*
RUN . /opt/ros/noetic/setup.sh && \
    git clone https://github.com/Microsoft/AirSim.git && \
    cd AirSim && \
    ./setup.sh && \
    ./build.sh && \
    cd ros && \
    catkin build && \
	pip install opencv-python && \
	pip install msgpack-rpc-python && \
	pip install airsim

RUN cd /root/ && \
	git clone https://github.com/pjreddie/darknet && \
	cd darknet && \
	make && \
	wget https://pjreddie.com/media/files/yolov3.weights
	

# Create a setup script
ADD setup.sh /root/setup.sh

# Set up .bashrc
RUN cp /etc/skel/.bashrc /root/.bashrc
RUN echo "source /root/setup.sh" >> /root/.bashrc
