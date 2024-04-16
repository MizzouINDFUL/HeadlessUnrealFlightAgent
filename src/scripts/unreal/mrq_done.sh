source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)
CURRLIFE=$(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)

echo "Ending life $CURRLIFE"

#send Ctrl+C to ROS-Bags in SIM session
tmux send-keys -t $SESSIONNAME:ROS-Bags C-c

#kill the AirSim window in SIM
docker kill unreal-launcher-airsim-ros
tmux kill-window -t $SESSIONNAME:AirSim

sleep 2

tmux send-keys -t $SESSIONNAME:ROS-Bags "mv *.bag ros.bag; rosbag info -y -k topics ros.bag > topics.yml; python3 /scripts/bag_extraction/init_bag_extraction.py" C-m

if [ $simulation_extract_on_end == true ]; then

    echo "Extracting bags"

    #create a new window called Bags-Extract in SIM session
    tmux new-window -t $SESSIONNAME -n Bags-Extract
    tmux send-keys -t $SESSIONNAME:Bags-Extract "source $UELAUNCHER_HOME/src/scripts/shared.sh; \
        eval $(parse_yaml $UELAUNCHER_HOME/config.yml)" C-m

    tmux send-keys -t $SESSIONNAME:Bags-Extract \
        "python3 $UELAUNCHER_HOME/src/scripts/bag_extraction/listen_extraction_requests.py; \
        tmux kill-window -t $SESSIONNAME:ROS-Bags; \
        docker kill $SESSIONNAME-yolo; \
        echo $(($CURRLIFE+0)); sleep 10;
        docker run -it --rm --gpus all --name $SESSIONNAME-yolo -v $UELAUNCHER_HOME/bags/$SIM_START_DATE/$(($CURRLIFE+0)):/session -v $UELAUNCHER_HOME/src/scripts/yolo:/scripts ultralytics/ultralytics python3 /scripts/run_yolo.py; \
        export NUM_LIVES=$(($NUM_LIVES+1)); \
        python3 $UELAUNCHER_HOME/src/scripts/send_restart_signal.py; \
        tmux select-window -t $SESSIONNAME:Orchestrator; \
        tmux kill-window -t $SESSIONNAME:Bags-Extract; \
        " C-m

    init_extraction_argument=""
    if [ $simulation_target_is_time == false ]; then
        init_extraction_argument="$simulation_target"
    fi

    #execute rosbag info -y -k topics ros.bag > topics.yml in ROS-Bags
    tmux send-keys -t $SESSIONNAME:ROS-Bags "mv *.bag ros.bag; rosbag info -y -k topics ros.bag > topics.yml; python3 /scripts/bag_extraction/init_bag_extraction.py $init_extraction_argument" C-m
else
    echo "Not extracting bags"
    tmux kill-window -t $SESSIONNAME:ROS-Bags
    export NUM_LIVES=$(($NUM_LIVES+1))
    python3 $UELAUNCHER_HOME/src/scripts/send_restart_signal.py
    tmux select-window -t $SESSIONNAME:Orchestrator
fi
