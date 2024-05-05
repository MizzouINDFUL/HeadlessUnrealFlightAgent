#!/bin/bash

#read the first argument - it tells us which life we are on
LIFE_NUM=$1

MAX_LIFE_NUM=$2

AGENT_ROOT_DIR="~/shared/src/agent"

#check if path exists
if [ ! -d "$AGENT_ROOT_DIR" ]; then
    #set it to THIS_SCRIPT_DR/src/agent
    THIS_SCRIPT_DIR=$(dirname "$0")
    AGENT_ROOT_DIR="$THIS_SCRIPT_DIR/src/agent"
fi

# tmux new-session -d -s AirSim -n AirSim 'source ~/setup.sh; sleep 3; cd ~/shared/launch; roslaunch airsim_all.launch; exec bash'
PYTHON_SCRIPT=$(find $AGENT_ROOT_DIR -name "*.py" -type f -print -quit)
python3 $PYTHON_SCRIPT $LIFE_NUM $MAX_LIFE_NUM
# tmux new-window -n Controller "source ~/setup.sh; sleep 9; python3 $PYTHON_SCRIPT $LIFE_NUM $MAX_LIFE_NUM; exec bash"
# tmux set-option -gw mouse on
# tmux attach-session -t AirSim
