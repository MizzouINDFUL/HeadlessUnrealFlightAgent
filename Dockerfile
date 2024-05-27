FROM nvidia/opengl:1.2-glvnd-devel-ubuntu20.04

#Install Unreal Engine 5.1
#Credits: Official Unreal Engine Dockerfile https://github.com/EpicGames/UnrealEngine/blob/5.4/Engine/Extras/Containers/Dockerfiles/linux/dev/Dockerfile
# COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine /home/ue4/UnrealEngine
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Binaries/DotNET/ /home/ue4/UnrealEngine/Engine/Binaries/DotNET/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Binaries/Linux/ /home/ue4/UnrealEngine/Engine/Binaries/Linux/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Binaries/ThirdParty/ /home/ue4/UnrealEngine/Engine/Binaries/ThirdParty/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Build /home/ue4/UnrealEngine/Engine/Build
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Content/ /home/ue4/UnrealEngine/Engine/Content/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Source/ /home/ue4/UnrealEngine/Engine/Source/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Programs/ /home/ue4/UnrealEngine/Engine/Programs/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Extras/ /home/ue4/UnrealEngine/Engine/Extras/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Intermediate/ /home/ue4/UnrealEngine/Engine/Intermediate/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Config/ /home/ue4/UnrealEngine/Engine/Config/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Shaders/ /home/ue4/UnrealEngine/Engine/Shaders/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Platforms/ /home/ue4/UnrealEngine/Engine/Platforms/
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/DerivedDataCache/ /home/ue4/UnrealEngine/Engine/DerivedDataCache/

COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/2D /home/ue4/UnrealEngine/Engine/Plugins/2D
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Animation /home/ue4/UnrealEngine/Engine/Plugins/Animation
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/AudioGameplayVolume /home/ue4/UnrealEngine/Engine/Plugins/AudioGameplayVolume
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Cameras /home/ue4/UnrealEngine/Engine/Plugins/Cameras
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Compression /home/ue4/UnrealEngine/Engine/Plugins/Compression
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Editor /home/ue4/UnrealEngine/Engine/Plugins/Editor
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/EnhancedInput /home/ue4/UnrealEngine/Engine/Plugins/EnhancedInput
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Experimental /home/ue4/UnrealEngine/Engine/Plugins/Experimental
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/FastBuildController /home/ue4/UnrealEngine/Engine/Plugins/FastBuildController
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Interchange /home/ue4/UnrealEngine/Engine/Plugins/Interchange
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/LightWeightInstancesEditor /home/ue4/UnrealEngine/Engine/Plugins/LightWeightInstancesEditor
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/MegascansPlugin /home/ue4/UnrealEngine/Engine/Plugins/MegascansPlugin
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Messaging /home/ue4/UnrealEngine/Engine/Plugins/Messaging
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/NetcodeUnitTest /home/ue4/UnrealEngine/Engine/Plugins/NetcodeUnitTest
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Performance /home/ue4/UnrealEngine/Engine/Plugins/Performance
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Protocols /home/ue4/UnrealEngine/Engine/Plugins/Protocols
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Runtime /home/ue4/UnrealEngine/Engine/Plugins/Runtime
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/ScriptPlugin /home/ue4/UnrealEngine/Engine/Plugins/ScriptPlugin
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Tests /home/ue4/UnrealEngine/Engine/Plugins/Tests
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/VirtualProduction /home/ue4/UnrealEngine/Engine/Plugins/VirtualProduction
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/XGEController /home/ue4/UnrealEngine/Engine/Plugins/XGEController
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/AI /home/ue4/UnrealEngine/Engine/Plugins/AI
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/AudioGameplay /home/ue4/UnrealEngine/Engine/Plugins/AudioGameplay
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/BlueprintFileUtils /home/ue4/UnrealEngine/Engine/Plugins/BlueprintFileUtils
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Compositing /home/ue4/UnrealEngine/Engine/Plugins/Compositing
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Developer /home/ue4/UnrealEngine/Engine/Plugins/Developer
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/EnginePlugin_A /home/ue4/UnrealEngine/Engine/Plugins/EnginePlugin_A
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Enterprise /home/ue4/UnrealEngine/Engine/Plugins/Enterprise
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/FX /home/ue4/UnrealEngine/Engine/Plugins/FX
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Importers /home/ue4/UnrealEngine/Engine/Plugins/Importers
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/JsonBlueprintUtilities /home/ue4/UnrealEngine/Engine/Plugins/JsonBlueprintUtilities
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Media /home/ue4/UnrealEngine/Engine/Plugins/Media
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/MeshPainting /home/ue4/UnrealEngine/Engine/Plugins/MeshPainting
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/MovieScene /home/ue4/UnrealEngine/Engine/Plugins/MovieScene
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Online /home/ue4/UnrealEngine/Engine/Plugins/Online
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Portal /home/ue4/UnrealEngine/Engine/Plugins/Portal
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/RenderGraphInsights /home/ue4/UnrealEngine/Engine/Plugins/RenderGraphInsights
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/ScriptGeneratorPlugin /home/ue4/UnrealEngine/Engine/Plugins/ScriptGeneratorPlugin
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Slate /home/ue4/UnrealEngine/Engine/Plugins/Slate
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/TraceUtilities /home/ue4/UnrealEngine/Engine/Plugins/TraceUtilities
COPY --from=ghcr.io/epicgames/unreal-engine:dev-5.1.0 --chown=ue4:ue4 /home/ue4/UnrealEngine/Engine/Plugins/Web /home/ue4/UnrealEngine/Engine/Plugins/Web

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
    # rm config.yml && \
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