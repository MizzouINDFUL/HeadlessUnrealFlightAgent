source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

sleep 5;
tmux new-window -t $SESSIONNAME -n AirSim "docker kill $SESSIONNAME-airsim-ros; ./src/airsim-ros/run.sh; exec bash";
tmux new-window -t $SESSIONNAME -n ROS-Bags "docker exec -it $SESSIONNAME-ros /bin/bash";
sleep 0.1;
#life num = $NUM_LIVES + 1
LIFE_NUM=$(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)
LIFE_NUM=$(($LIFE_NUM+1))

topics_regex=$($UELAUNCHER_HOME/src/scripts/bag_extraction/get_rosbag_topics_regex.sh)
tmux send-keys -t $SESSIONNAME:ROS-Bags "source /opt/ros/noetic/setup.bash; cd /session/; mkdir $LIFE_NUM; cd $LIFE_NUM; rosbag record -a; exec bash" C-m
# tmux send-keys -t $SESSIONNAME:ROS-Bags "source /opt/ros/noetic/setup.bash; cd /session/; mkdir $LIFE_NUM; cd $LIFE_NUM; rosbag record -e \"$topics_regex\" ros.bag; exec bash" C-m

sleep 30;