THISFOLDER=$(dirname $(readlink -f $0))
source $THISFOLDER/src/scripts/shared.sh
eval $(parse_yaml $1)

#if tmp folder doesnt exist, create it
if [ ! -d "$HOME_DIR/tmp" ]; then
    mkdir $HOME_DIR/tmp
fi

SESSIONBASENAME=$(yq e '.session.basename' $1)

#Handling multiple sessions in parallel
all_sessions=$(tmux list-sessions -F "#{session_name}" 2>/dev/null)
num_sim_sessions=$(echo "$all_sessions" | grep -c "^$SESSIONBASENAME")

SESSIONNAME="$SESSIONBASENAME$num_sim_sessions"

#saving an intermediate yml file that stores the parameters for the imulation as well as its current state
cp $1 $HOME_DIR/tmp/$SESSIONNAME-config.yml

SESSIONBASENAME=$(yq e '.session.basename' $1)
MAXSESSIONS=$(yq e '.session.max' $1)
START_LIFE=$(yq e '.session.start_life' $1)
END_LIFE=$(yq e '.session.end_life' $1)

# yq e ".sessionname = $SESSIONNAME" -i $HOME_DIR/tmp/$SESSIONNAME-config.yml
yq e ".current_life = $START_LIFE" -i $HOME_DIR/tmp/$SESSIONNAME-config.yml
yq e ".sessionname = \"$SESSIONNAME\"" -i $HOME_DIR/tmp/$SESSIONNAME-config.yml

#Get available ports for each entry in 'ports_to_reserve' and write them to the config file
python3 src/scripts/find_available_ports.py $SESSIONBASENAME $num_sim_sessions

#this will generate a txt file with all the arguments we want to pass to UnrealEditor-Cmd.sh
python3 src/scripts/parse_unreal_arguments.py $HOME_DIR/tmp/$SESSIONNAME-config.yml

#cancel if there are too many sessions (and $MAXSESSIONS is not equal to -1)
if [ "$MAXSESSIONS" != "-1" ]; then
   if [ "$num_sim_sessions" -ge "$MAXSESSIONS" ]; then
      echo "Maximum number of sessions reached. Please close some sessions before starting a new one."
      exit 1
   fi
fi

UELAUNCHER_HOME=$HOME_DIR
SIM_START_DATE=$(date +%Y-%m-%d_%H-%M-%S)
SESSIONROOT=$SESSIONBASENAME

init_unreal_script="init_unreal.sh"
if [ $unreal_use_docker == true ]; then
    init_unreal_script="init_unreal_docker.sh"
fi

#check if ./logs exists. If it doesn't, create it
if [ ! -d "$HOME_DIR/src/logs" ]; then
    mkdir $HOME_DIR/src/logs
fi

if [ -d "$HOME_DIR/bags/" ]; then
    mkdir $HOME_DIR/bags/
fi

#check if ./bags/$SESSIONROOT exists. If it doesn't, create it
if [ ! -d "$HOME_DIR/bags/$SESSIONROOT" ]; then
    mkdir $HOME_DIR/bags/$SESSIONROOT
fi

touch $HOME_DIR/src/logs/$SESSIONNAME-Unreal.log
echo "" > $HOME_DIR/src/logs/$SESSIONNAME-Unreal.log

#go to project directory and delete Saved/Crashes and Saved/Autosaves. If the game wasn't closed properly before,
#this next run will propmt us with a "Restore Packages" window. That window is not interactive in headless mode and so the game will not start
rm -r $unreal_project_path/Saved/Crashes > /dev/null 2>&1
rm -r $unreal_project_path/Saved/Autosaves > /dev/null 2>&1

#get the name of the game project from the config file parameter $unreal_project_path. Get the name of the last folder in that path
GAME_PROJECT_NAME=$(echo $unreal_project_path | rev | cut -d'/' -f1 | rev)
echo "GAME_PROJECT_NAME: $GAME_PROJECT_NAME"

EXPORT_COMMAND="export SESSIONNAME=$SESSIONNAME; export UELAUNCHER_HOME=$UELAUNCHER_HOME; export SIM_START_DATE=$SIM_START_DATE; export SESSIONROOT=$SESSIONROOT; export GAME_PROJECT_NAME=$GAME_PROJECT_NAME"

#This will initiate the basic structure of the tmux session:
# - Unreal Engine: This is where we are talking to the instance of UE: both compilation adn the launch of the project are happening here
# - tellunreal: a window where we can send commands to the Unreal Engine instance. For example: tellunreal 'py print("hello world")' will print a hello world in the Unreal Engine cmd
# - Orchestartor: a window that tracks the logs of the current Unreal Engine instance and launches scripts whenever it sees trigger phrases in it.
# - ROS: a window that launches the docker container that runs the ROS side of the simulation
 
tmux -f "$HOME_DIR/src/tmux.conf" new-session -d -s $SESSIONNAME -n UnrealEngine
tmux new-window -t $SESSIONNAME:1 -n tellunreal
tmux new-window -t $SESSIONNAME:2 -n Orchestrator
tmux new-window -t $SESSIONNAME:3 -n ROS

tmux send-keys -t $SESSIONNAME:UnrealEngine "$EXPORT_COMMAND" C-m
tmux send-keys -t $SESSIONNAME:tellunreal "$EXPORT_COMMAND" C-m
tmux send-keys -t $SESSIONNAME:Orchestrator "$EXPORT_COMMAND" C-m
tmux send-keys -t $SESSIONNAME:ROS "$EXPORT_COMMAND" C-m

tmux pipe-pane -o -t $SESSIONNAME:UnrealEngine "tee -a $HOME_DIR/src/logs/$SESSIONNAME-Unreal.log >> $HOME_DIR/tmp/$SIM_START_DATE-Unreal.log"
tmux send-keys -t $SESSIONNAME:UnrealEngine "$HOME_DIR/src/scripts/unreal/$init_unreal_script" C-m
bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/start_tellunreal.sh

ROS_PORT=$(yq e '.ports_to_reserve[3].rosbridge_listener' $HOME_DIR/tmp/$SESSIONNAME-config.yml)
ENABLE_ROS=$(yq e '.ros.enable' $HOME_DIR/tmp/$SESSIONNAME-config.yml)
if [ "$ENABLE_ROS" = true ]; then
    ROS_USE_DOCKER=$(yq e '.ros.use_docker' $HOME_DIR/tmp/$SESSIONNAME-config.yml)
    if [ "$ROS_USE_DOCKER" = true ]; then
        ROS_IMAGE=$(yq e '.ros.docker_image' $HOME_DIR/tmp/$SESSIONNAME-config.yml)

        tmux send-keys -t $SESSIONNAME:ROS "docker kill $SESSIONNAME-airsim-ros" C-m
        tmux send-keys -t $SESSIONNAME:ROS "docker run -it --net host -e DISPLAY=$DISPLAY \
        -v ./src/airsim-ros/shared:/root/shared \
        -v $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml:/config.yml \
        -v $UELAUNCHER_HOME/bags/$SESSIONROOT/:/session \
        -v $UELAUNCHER_HOME/src/scripts/:/scripts \
        --rm --name $SESSIONNAME-airsim-ros \
        $ROS_IMAGE /init-rosbridge.sh $ROS_PORT $SESSIONNAME" C-m
    else
        tmux send-keys -t $SESSIONNAME:ROS "source $HOME_DIR/src/airsim-ros/init_rosbridge.sh $ROS_PORT $SESSIONNAME" C-m
    fi

fi

bind_script_to_event "External Command Line object is initialized" $HOME_DIR/src/scripts/unreal/init_viewport_capture.sh

if [ $unreal_start_game == true ]; then
    bind_script_to_event "VIEWPORT CAPTURE READY" $HOME_DIR/src/scripts/unreal/start_game.sh
fi

#When "Bringing up level for play took" appears in the logs, launch unreal-launcher-airsim-ros docker
bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/start_airsim_rosbag.sh
bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/unreal/post_start_game.sh

if [ $simulation_target -gt -1 ]; then
    # bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/unreal/stop_game.sh true
    bind_script_to_event "MRQ SIM FINISHED" $HOME_DIR/src/scripts/unreal/mrq_done.sh true
    bind_script_to_event "Bringing up level for play took" $HOME_DIR/src/scripts/listen_restart_signal.sh
fi

tmux set -g mouse on
tmux attach-session -t $SESSIONNAME:ROS