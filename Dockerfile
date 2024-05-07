FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

#Install Unreal Engine 5.1
#Credits: Official Unreal Engine Dockerfile https://github.com/EpicGames/UnrealEngine/blob/5.4/Engine/Extras/Containers/Dockerfiles/linux/dev/Dockerfile
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine /home/ue4/UnrealEngine

# Disable interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Enable CUDA support for NVIDIA GPUs (even when not using a CUDA base image), since evidently some versions of UE unconditionally assume
# `libcuda.so.1` exists when using the NVIDIA proprietary drivers, and will fail to initialise the Vulkan RHI if it is missing
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},compute

# Add the "display" driver capability for NVIDIA GPUs
# (This allows us to run the Editor from an interactive container by bind-mounting the host system's X11 socket)
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},display

# Enable NVENC support for use by Unreal Engine plugins that depend on it (e.g. Pixel Streaming)
# (Note that adding `video` seems to implicitly enable `compute` as well, but we include separate directives here to clearly indicate the purpose of both)
ENV NVIDIA_DRIVER_CAPABILITIES ${NVIDIA_DRIVER_CAPABILITIES},video

# Install our build prerequisites
RUN apt-get update && apt-get install -y --no-install-recommends \
		build-essential \
		ca-certificates \
		curl \
		git \
		git-lfs \
		gpg-agent \
		python3 \
		python3-dev \
		python3-pip \
		shared-mime-info \
		software-properties-common \
		sudo \
		tzdata \
		unzip \
		xdg-user-dirs \
		xdg-utils \
		zip && \
	rm -rf /var/lib/apt/lists/* && echo '' && echo 'RUN directive complete. Docker will now commit the filesystem layer to disk.' && echo 'Note that for large filesystem layers this can take quite some time.' && echo 'Performing filesystem layer commit...' && echo ''

# Install the X11 runtime libraries required by CEF so we can cook Unreal Engine projects that use the WebBrowserWidget plugin
# (Starting in Unreal Engine 5.0, we need these installed before creating an Installed Build to prevent cooking failures related to loading the Quixel Bridge plugin)
RUN apt-get update && apt-get install -y --no-install-recommends \
			libasound2 \
			libatk1.0-0 \
			libatk-bridge2.0-0 \
			libcairo2 \
			libfontconfig1 \
			libfreetype6 \
			libgbm1 \
			libglu1 \
			libnss3 \
			libnspr4 \
			libpango-1.0-0 \
			libpangocairo-1.0-0 \
			libsm6 \
			libxcomposite1 \
			libxcursor1 \
			libxdamage1 \
			libxi6 \
			libxkbcommon-x11-0 \
			libxrandr2 \
			libxrender1 \
			libxss1 \
			libxtst6 \
			libxv1 \
			x11-xkb-utils \
			xauth \
			xfonts-base \
			xkb-data && \
	rm -rf /var/lib/apt/lists/* && echo '' && echo 'RUN directive complete. Docker will now commit the filesystem layer to disk.' && echo 'Note that for large filesystem layers this can take quite some time.' && echo 'Performing filesystem layer commit...' && echo ''

# Unreal refuses to run as the root user, so create a non-root user with no password and allow them to run commands using sudo
RUN useradd --create-home --home /home/ue4 --shell /bin/bash --uid 1000 ue4 && \
	passwd -d ue4 && \
	usermod -a -G audio,video,sudo ue4 && echo '' && echo 'RUN directive complete. Docker will now commit the filesystem layer to disk.' && echo 'Note that for large filesystem layers this can take quite some time.' && echo 'Performing filesystem layer commit...' && echo ''
USER ue4

# Install Python 3.12, which is required by ushell
USER root
RUN add-apt-repository -y ppa:deadsnakes/ppa && \
	apt-get update && \
	apt-get install -y --no-install-recommends python3.12 python3.12-venv && \
	rm -rf /var/lib/apt/lists/* && echo '' && echo 'RUN directive complete. Docker will now commit the filesystem layer to disk.' && echo 'Note that for large filesystem layers this can take quite some time.' && echo 'Performing filesystem layer commit...' && echo ''
USER ue4
WORKDIR /home/ue4/UnrealEngine

USER root

RUN EDITOR_TARGET="UnrealEditor" && \
	./Engine/Build/BatchFiles/Linux/Build.sh "$EDITOR_TARGET" Linux Development -SkipBuild && \
	mkdir -p ./Engine/Programs/AutomationTool/Saved && \
	chmod a+rw ./Engine/Programs/AutomationTool/Saved && echo '' && echo 'RUN directive complete. Docker will now commit the filesystem layer to disk.' && echo 'Note that for large filesystem layers this can take quite some time.' && echo 'Performing filesystem layer commit...' && echo ''


# Enable Vulkan support for NVIDIA GPUs
USER root
RUN apt-get update && apt-get install -y --no-install-recommends libvulkan1 && \
	rm -rf /var/lib/apt/lists/* && \
	VULKAN_API_VERSION=`dpkg -s libvulkan1 | grep -oP 'Version: [0-9|\.]+' | grep -oP '[0-9|\.]+'` && \
	mkdir -p /etc/vulkan/icd.d/ && \
	echo \
	"{\
		\"file_format_version\" : \"1.0.0\",\
		\"ICD\": {\
			\"library_path\": \"libGLX_nvidia.so.0\",\
			\"api_version\" : \"${VULKAN_API_VERSION}\"\
		}\
	}" > /etc/vulkan/icd.d/nvidia_icd.json && echo '' && echo 'RUN directive complete. Docker will now commit the filesystem layer to disk.' && echo 'Note that for large filesystem layers this can take quite some time.' && echo 'Performing filesystem layer commit...' && echo ''

#Install ROS Noetic
# Set timezone
ENV TZ=America/Chicago
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ/etc/timezone

# Install basic dependencies
RUN apt-get update && apt-get install -y \
    cmake \
    curl \
    git \
    gnupg \
    iproute2 \
    iputils-ping \
    libboost-iostreams-dev \
    libboost-numpy-dev \
    libboost-python-dev \
    libtbb-dev \
    libblosc-dev \
    mesa-utils \
    python3-pip \
    tmux \
    vim \
    wget \
    x11-apps

# Install ROS packages
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full

WORKDIR /root

# Install common Python packages
RUN pip install numpy==1.22 open3d scipy av pyproj

# Set up rosdep
RUN apt-get install -y python3-catkin-tools python3-osrf-pycommon python3-rosdep
RUN rosdep init && rosdep update

#Install rosbridge
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
# ADD setup.sh /root/setup.sh

# # Set up .bashrc
# RUN cp /etc/skel/.bashrc /root/.bashrc
# RUN echo "source /root/setup.sh" >> /root/.bashrc

# COPY init-rosbridge.sh .

# RUN chmod +x init-rosbridge.sh

# Install AirSim
RUN apt-get update && apt-get install -y git ros-noetic-catkin ros-noetic-tf2-sensor-msgs ros-noetic-tf2-geometry-msgs ros-noetic-mavros*
RUN . /opt/ros/noetic/setup.sh && \
    git clone https://github.com/CodexLabsLLC/Colosseum.git && \
    cd Colosseum && \
    ./setup.sh && \
    ./build.sh && \
    pip install opencv-python && \
    pip install msgpack-rpc-python && \
    pip install airsim && \
    pip install cvbridge3 && \
    pip install pyyaml && \
    cd ros && \
    catkin build && \
    mkdir /root/AirSim && \
    ln -s /Colosseum/ros /root/AirSim

# Create a setup script
RUN touch /root/setup.sh
RUN echo "source /root/AirSim/ros/devel/setup.bash" >> /root/setup.sh

# Set up .bashrc
RUN cp /etc/skel/.bashrc /root/.bashrc
RUN echo "source /root/setup.sh" >> /root/.bashrc



#Install MizSIM

RUN cd /home && \
    git config --global --add safe.directory /home/HeadlessUnrealFlightAgent && \
    git clone https://github.com/MizzouINDFUL/HeadlessUnrealFlightAgent.git && \
    cd HeadlessUnrealFlightAgent && \
    #remove the default config file - we expect users to link their own
    rm config.yml && \
	rm Dockerfile && \
	rm setup.sh && \
    mkdir bags && \
    mkdir /projects && \
    cp /home/HeadlessUnrealFlightAgent/src/airsim-ros/init-rosbridge.sh /init-rosbridge.sh && \
    chmod a+x /init-rosbridge.sh


#Install tmux and yq
RUN apt-get update && apt-get install -y tmux && \
    sudo add-apt-repository ppa:rmescandon/yq && \
    sudo apt update && \
    sudo apt install yq -y

# set the starting directcoy to HeadlessUnrealFlightAgent
WORKDIR /home/HeadlessUnrealFlightAgent