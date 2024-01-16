#!/bin/bash

#read the first argument - it tells us which life we are on
LIFE_NUM=$1

MAX_LIFE_NUM=$2

tmux new-session -d -s AirSim -n AirSim 'source ~/setup.sh; sleep 3; cd ~/shared/launch; roslaunch airsim_all.launch; exec bash'
PYTHON_SCRIPT=$(find ~/shared/src/agent -name "*.py" -type f -print -quit)
tmux new-window -n Controller "source ~/setup.sh; sleep 9; python3 $PYTHON_SCRIPT $LIFE_NUM $MAX_LIFE_NUM; exec bash"
tmux set-option -gw mouse on
tmux attach-session -t AirSim
