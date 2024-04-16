source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

sleep 5;

#create this window only if $simulation_start_airsim is true
if [ $simulation_start_airsim == true ]; then
    tmux new-window -t $SESSIONNAME -n AirSim "docker kill $SESSIONNAME-airsim-ros; ./src/airsim-ros/run.sh; exec bash";
fi
if [ $simulation_store_in_bag == true ]; then
    tmux new-window -t $SESSIONNAME -n ROS-Bags "docker exec -it $SESSIONNAME-ros /bin/bash";
fi
sleep 0.1;
#life num = $NUM_LIVES + 1
LIFE_NUM=$(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)
LIFE_NUM=$(($LIFE_NUM+1))

if [ $simulation_store_in_bag == true ]; then
    topics_regex=$($UELAUNCHER_HOME/src/scripts/bag_extraction/get_rosbag_topics_regex.sh)
    tmux send-keys -t $SESSIONNAME:ROS-Bags "source /opt/ros/noetic/setup.bash; cd /session/; mkdir $LIFE_NUM; cd $LIFE_NUM; sleep $simulation_record_delay; rosbag record -e '/unreal_ros/(.*)'; exec bash" C-m
fi

sleep 30;