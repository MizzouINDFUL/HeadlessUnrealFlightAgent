source $UELAUNCHER_HOME/src/scripts/shared.sh

LIFE_NUM=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
MAX_LIVES=$(yq e '.session.end_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
RUN_AIRSIM=$(yq e '.airsim.enable' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
AIRSIM_USE_DOCKER=$(yq e '.airsim.use_docker' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
AIRSIM_IMAGE=$(yq e '.airsim.docker_image' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
STORE_DATA_IN_BAGS=$(yq e '.ros.enable_rosbag' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
ROS_USE_DOCKER=$(yq e '.ros.use_docker' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
SESSION_ROOTFOLDER=$(yq e '.session.basename' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

echo "Starting AirSim with $LIFE_NUM lives and $MAX_LIVES max lives";
sleep 5;

if [ $RUN_AIRSIM == true ]; then
    if [ $AIRSIM_USE_DOCKER == true ]; then
        tmux new-window -t $SESSIONNAME -n AirSim "docker exec -ti $SESSIONNAME-airsim-ros bash -c \"~/shared/start_tmux_windows.sh $LIFE_NUM $MAX_LIVES \"; exec bash";
    else
        tmux new-window -t $SESSIONNAME -n AirSim "bash -c \"$UELAUNCHER_HOME/src/airsim-ros/shared/start_tmux_windows.sh $LIFE_NUM $MAX_LIVES $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml \"; exec bash";
    fi

    LOG_AIRSIM=$(yq e '.airsim.log' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
    if [ "$LOG_AIRSIM" == true ]; then
        tmux pipe-pane -o -t $SESSIONNAME:AirSim "tee -a $UELAUNCHER_HOME/src/logs/$SESSIONNAME-AirSim.log >> $UELAUNCHER_HOME/tmp/$SIM_START_DATE-AirSim.log"
    fi
fi

sleep 30;