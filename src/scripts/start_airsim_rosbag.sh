source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

LIFE_NUM=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
MAX_LIVES=$(yq e '.session.end_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
RUN_AIRSIM=$(yq e '.airsim.enable' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
AIRSIM_USE_DOCKER=$(yq e '.airsim.use_docker' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
AIRSIM_IMAGE=$(yq e '.airsim.docker_image' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
STORE_DATA_IN_BAGS=$(yq e '.ros.enable_rosbag' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
ROS_USE_DOCKER=$(yq e '.ros.use_docker' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
SESSION_ROOTFOLDER=$(yq e '.session.basename' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

sleep 5;

#create this window only if $simulation_start_airsim is true
if [ $RUN_AIRSIM == true ]; then
    if [ $AIRSIM_USE_DOCKER == true ]; then
        tmux new-window -t $SESSIONNAME -n AirSim "docker exec -ti $SESSIONNAME-airsim-ros bash -c \"~/shared/start_tmux_windows.sh $LIFE_NUM $MAX_LIVES \"; exec bash";
    else
        tmux new-window -t $SESSIONNAME -n AirSim "bash -c \"$UELAUNCHER_HOME/src/airsim-ros/shared/start_tmux_windows.sh $LIFE_NUM $MAX_LIVES $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml \"; exec bash";
    fi
fi
if [ $STORE_DATA_IN_BAGS == true ]; then
    if [ $ROS_USE_DOCKER == true ]; then
        tmux new-window -t $SESSIONNAME -n ROS-Bags "docker exec -it $SESSIONNAME-airsim-ros /bin/bash";
        sleep 0.1
        tmux send-keys -t $SESSIONNAME:ROS-Bags "source /opt/ros/noetic/setup.bash; cd /session/; mkdir $LIFE_NUM; cd $LIFE_NUM; sleep $simulation_record_delay; rosbag record -e '/$SESSIONNAME/unreal_ros/(.*)'; exec bash" C-m
    else
        tmux new-window -t $SESSIONNAME -n ROS-Bags "bash -c \"source /opt/ros/noetic/setup.bash; cd ./bags/$SESSION_ROOTFOLDER; mkdir $LIFE_NUM; cd $LIFE_NUM; sleep $simulation_record_delay; rosbag record -e '/$SESSIONNAME/unreal_ros/(.*)'; exec bash\"";
    fi
fi

sleep 30;