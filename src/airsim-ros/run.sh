docker run -it --rm --net host -e DISPLAY=$DISPLAY -v ./src/airsim-ros/shared:/root/shared --name airsim-ros airsim-ros bash -c "~/shared/start_tmux_windows.sh"
