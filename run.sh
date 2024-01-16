THISFOLDER=$(dirname $(readlink -f $0))
source $THISFOLDER/src/scripts/shared.sh
eval $(parse_yaml $HOME_DIR/config.yml)

#if $THISFOLDER/agent doesnt exist, users havent ran setup.sh first
if [ ! -d "$THISFOLDER/agent" ]; then
    echo "Please run setup.sh first."
    exit 1
fi

#if tmp folder doesnt exist, create it
if [ ! -d "$HOME_DIR/tmp" ]; then
    mkdir $HOME_DIR/tmp
fi

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

init_unreal_script="init_unreal.sh"

if [ $unreal_use_docker == true ]; then
    init_unreal_script="init_unreal_docker.sh"
fi

mkdir ./bags/$SIM_START_DATE

echo "" > $HOME_DIR/src/logs/Unreal.log
echo "" > $HOME_DIR/src/plugins_link/CommandLineExternal/command.txt

#get the name of the game project from the config file parameter $unreal_project_path. Get the name of the last folder in that path
export GAME_PROJECT_NAME=$(echo $unreal_project_path | rev | cut -d'/' -f1 | rev)
echo "GAME_PROJECT_NAME: $GAME_PROJECT_NAME"

#This will initiate the basic structure of tmux:
# - Unreal Engine: This is where we are talking to the instance of UE: both compilation adn the launch of the project are happening here
# - tellunreal: a window where we can send commands to the Unreal Engine instance. For example: tellunreal 'py print("hello world")' will execute a hello world command in the Unreal Engine instance
# - Orchestartor: a window that tracks the logs of the current Unreal Engine instance and launches scripts whenever it sees trigger phrases in it. Initially this will be create with nothing running in it. But users can new panes to this that will run actual listener scripts whenever it is needed.

tmux -f "$UELAUNCHER_HOME/src/tmux.conf" new-session -d -s $SESSIONNAME -n UnrealEngine
tmux new-window -t $SESSIONNAME:1 -n tellunreal
tmux new-window -t $SESSIONNAME:2 -n Orchestrator
tmux new-window -t $SESSIONNAME:3 -n ROS

if [ $unreal_separate_session == false ]; then
    tmux pipe-pane -o -t $SESSIONNAME:UnrealEngine "tee -a $HOME_DIR/src/logs/Unreal.log >> $HOME_DIR/tmp/$SIM_START_DATE-Unreal.log"
    tmux send-keys -t $SESSIONNAME:UnrealEngine "$HOME_DIR/src/scripts/unreal/$init_unreal_script" C-m
    bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/start_tellunreal.sh
else
    #check if the tmux session for the game project already exists. If it does, export the outputs of that session to the Unreal.log file
    if [ $(tmux list-sessions -F "#{session_name}" 2>/dev/null | grep -c "^$GAME_PROJECT_NAME-$SESSIONINDEX") -gt 0 ]; then
    echo "A runnning instance of Unreal Engine is found. Connecting to that..."
        sleep 1
        tmux pipe-pane -o -t $GAME_PROJECT_NAME-$SESSIONINDEX:UnrealEngine "tee -a $HOME_DIR/src/logs/Unreal.log >> $HOME_DIR/tmp/$SIM_START_DATE-Unreal.log"
        $UELAUNCHER_HOME/src/scripts/unreal/start_tellunreal.sh
        sleep 2
        tmux send-keys -t $SESSIONNAME:tellunreal "tmux pipe-pane -o -t $GAME_PROJECT_NAME-$SESSIONINDEX:UnrealEngine \"cat >> $UELAUNCHER_HOME/src/logs/Unreal.log\"" C-m
        tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py print(\"External Command Line object is initialized\")'" C-m
        sleep 1
    else
        #if the tmux session for the game project does not exist, create it
        tmux new-session -d -s $GAME_PROJECT_NAME-$SESSIONINDEX -n UnrealEngine
        tmux pipe-pane -o -t $GAME_PROJECT_NAME-$SESSIONINDEX:UnrealEngine "tee -a $HOME_DIR/src/logs/Unreal.log >> $HOME_DIR/tmp/$SIM_START_DATE-Unreal.log"
        tmux send-keys -t $GAME_PROJECT_NAME-$SESSIONINDEX:UnrealEngine "$HOME_DIR/src/scripts/unreal/$init_unreal_script" C-m
        bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/start_tellunreal.sh
    fi
    tmux kill-window -t $SESSIONNAME:UnrealEngine
fi

tmux send-keys -t $SESSIONNAME:ROS "docker kill $SESSIONNAME-ros; docker run -it --net host -v $UELAUNCHER_HOME/bags/$SIM_START_DATE/:/session -v $UELAUNCHER_HOME/src/scripts/:/scripts --rm --name $SESSIONNAME-ros unreal-launcher-ros" C-m

#when External Command Line object is initialized, create an alias called 'tellunreal' inside tellunreal tmux window that will send the argument to command.txt in src/plugins_link/CommandLineExternal
bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/init_viewport_capture.sh

if [ $unreal_start_game == true ]; then
    bind_script_to_event "VIEWPORT CAPTURE READY" $HOME_DIR/src/scripts/unreal/start_game.sh
fi

#When "Bringing up level for play took" appears in the logs, launch unreal-launcher-airsim-ros docker
bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/start_airsim_rosbag.sh

if [ $simulation_time -gt -1 ]; then
    bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/unreal/stop_game.sh true
    bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/listen_restart_signal.sh
fi


tmux set -g mouse on
tmux attach-session -t $SESSIONNAME:ROS