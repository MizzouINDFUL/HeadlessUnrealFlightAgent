source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

CURRLIFE=$(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)
echo "Ending life $CURRLIFE in $simulation_time seconds"
sleep $simulation_time

#send Ctrl+C to ROS-Bags in SIM session
tmux send-keys -t SIM:ROS-Bags C-c

#kill the AirSim window in SIM
docker kill airsim-ros
tmux kill-window -t SIM:AirSim

sleep 5

#send 'py unreal.MindfulLib.stop_life()' to tellunreal window in SIM session
tmux send-keys -t SIM:tellunreal "tellunreal 'py unreal.MindfulLib.stop_life()'" C-m

#create a new window called Bags-Extract in SIM session
tmux new-window -t SIM -n Bags-Extract
tmux send-keys -t SIM:Bags-Extract "source $UELAUNCHER_HOME/src/scripts/shared.sh; \
    eval $(parse_yaml $UELAUNCHER_HOME/config.yml)" C-m

mux send-keys -t SIM:Bags-Extract "python3 $UELAUNCHER_HOME/src/scripts/bag_extraction/listen_extraction_requests.py" C-m

tmux send-keys -t SIM:Bags-Extract \
    "python3 $UELAUNCHER_HOME/src/scripts/bag_extraction/listen_extraction_requests.py; \
    tmux kill-window -t SIM:ROS-Bags; \
    docker kill yolo; \
    docker run -it --rm --gpus all --name yolo -v $UELAUNCHER_HOME/bags/$SIM_START_DATE/$(($NUM_LIVES+1)):/session -v $UELAUNCHER_HOME/src/scripts/yolo:/scripts ultralytics/ultralytics python3 /scripts/run_yolo.py; \
    export NUM_LIVES=$(($NUM_LIVES+1)); \
    python3 $UELAUNCHER_HOME/src/scripts/send_restart_signal.py; \
    tmux select-window -t SIM:Orchestrator; \
    tmux kill-window -t SIM:Bags-Extract; \
    " C-m

#$UELAUNCHER_HOME/bags/$SIM_START_DATE/:/session 

#execute rosbag info -y -k topics ros.bag > topics.yml in ROS-Bags
tmux send-keys -t SIM:ROS-Bags "mv *.bag ros.bag; rosbag info -y -k topics ros.bag > topics.yml; python3 /scripts/bag_extraction/init_bag_extraction.py" C-m