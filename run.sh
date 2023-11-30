THISFOLDER=$(dirname $(readlink -f $0))
source $THISFOLDER/src/scripts/shared.sh
eval $(parse_yaml $HOME_DIR/config.yml)

#Handling multiple sessions

all_sessions=$(tmux list-sessions -F "#{session_name}" 2>/dev/null)
num_sim_sessions=$(echo "$all_sessions" | grep -c "^$session_basename")

#cancel if there are too many sessions (and $session_max is not equal to -1)
if [ "$session_max" != "-1" ]; then
   if [ "$num_sim_sessions" -ge "$session_max" ]; then
      echo "Maximum number of sessions reached. Please close some sessions before starting a new one."
      exit 1
   fi
fi

SESSIONNAME=$session_basename
SESSIONNAME="$SESSIONNAME$num_sim_sessions"

export SESSIONNAME=$SESSIONNAME
export SESSIONINDEX=$num_sim_sessions

export UELAUNCHER_HOME=$HOME_DIR
export SIM_START_DATE=$(date +%Y-%m-%d_%H-%M-%S)
export NUM_LIVES=0

mkdir ./bags/$SIM_START_DATE

echo "" > $HOME_DIR/src/logs/Unreal.log
echo "" > $HOME_DIR/src/plugins_link/CommandLineExternal/command.txt

#This will initiate the basic structure of tmux:
# - Unreal Engine: This is where we are talking to the instance of UE: both compilation adn the launch of the project are happening here
# - tellunreal: a window where we can send commands to the Unreal Engine instance. For example: tellunreal 'py print("hello world")' will execute a hello world command in the Unreal Engine instance
# - Orchestartor: a window that tracks the logs of the current Unreal Engine instance and launches scripts whenever it sees trigger phrases in it. Initially this will be create with nothing running in it. But users can new panes to this that will run actual listener scripts whenever it is needed.

tmux -f "$UELAUNCHER_HOME/src/tmux.conf" new-session -d -s $SESSIONNAME -n UnrealEngine
tmux new-window -t $SESSIONNAME:1 -n tellunreal
tmux new-window -t $SESSIONNAME:2 -n Orchestrator
tmux new-window -t $SESSIONNAME:3 -n ROS

tmux pipe-pane -o -t $SESSIONNAME:UnrealEngine "cat >> $HOME_DIR/src/logs/Unreal.log"

#preparing to launch Unreal Engine
tmux send-keys -t $SESSIONNAME:UnrealEngine "$HOME_DIR/src/scripts/unreal/init_unreal.sh" C-m

tmux send-keys -t $SESSIONNAME:ROS "docker kill ros; docker run -it --net host -v $UELAUNCHER_HOME/bags/$SIM_START_DATE/:/session -v $UELAUNCHER_HOME/src/scripts/:/scripts --rm --name ros ros:server" C-m

#when External Command Line object is initialized, create an alias called 'tellunreal' inside tellunreal tmux window that will send the argument to command.txt in src/plugins_link/CommandLineExternal
bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/start_tellunreal.sh
bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/init_viewport_capture.sh

if [ $unreal_start_game == true ]; then
    bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/start_game.sh
fi

#When "Bringing up level for play took" appears in the logs, launch airsim-ros docker
bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/start_airsim_rosbag.sh

if [ $simulation_time -gt -1 ]; then
    bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/unreal/stop_game.sh true
    bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/listen_restart_signal.sh
fi


tmux set -g mouse on
tmux attach-session -t $SESSIONNAME:0