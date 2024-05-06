#!/bin/bash

#read the first argument - it tells us which life we are on
LIFE_NUM=$1

MAX_LIFE_NUM=$2

AGENT_ROOT_DIR="~/shared/src/agent"

CONFIG_DIR="/config.yml"

#if another argument is passed, it is the path to the config file
if [ ! -z "$3" ]; then
    CONFIG_DIR=$3
fi

#check if path exists
if [ ! -d "$AGENT_ROOT_DIR" ]; then
    #if it doesn't, we are not running AirSim inside of a docker container
    #set it to THIS_SCRIPT_DR/src/agent
    THIS_SCRIPT_DIR=$(dirname "$0")
    AGENT_ROOT_DIR="$THIS_SCRIPT_DIR/src/agent"
fi

# tmux new-session -d -s AirSim -n AirSim 'source ~/setup.sh; sleep 3; cd ~/shared/launch; roslaunch airsim_all.launch; exec bash'
PYTHON_SCRIPT=$(find $AGENT_ROOT_DIR -name "*.py" -type f -print -quit)
python3 $PYTHON_SCRIPT $LIFE_NUM $MAX_LIFE_NUM $CONFIG_DIR
# tmux new-window -n Controller "source ~/setup.sh; sleep 9; python3 $PYTHON_SCRIPT $LIFE_NUM $MAX_LIFE_NUM; exec bash"
# tmux set-option -gw mouse on
# tmux attach-session -t AirSim
