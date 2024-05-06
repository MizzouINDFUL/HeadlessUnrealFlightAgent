source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

CURRLIFE=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
RESTART_SIGNAL_PORT=$(yq e '.ports_to_reserve[1].life_restart_listener' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
EXTRACTION_PORT=$(yq e '.ports_to_reserve[2].rosbag_extraction_listener' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
SESSIONROOT=$(yq e '.session.basename' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
SHOULD_EXTRACT=$(yq e '.ros.extract_bags' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
ROS_USE_DOCKER=$(yq e '.ros.use_docker' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

echo "Ending life $CURRLIFE"

#send Ctrl+C to ROS-Bags in SIM session
tmux send-keys -t $SESSIONNAME:ROS-Bags C-c

#kill the AirSim window in SIM
tmux send-keys -t $SESSIONNAME:AirSim C-c
sleep 0.2
tmux kill-window -t $SESSIONNAME:AirSim

sleep 2

if [ $SHOULD_EXTRACT == true ]; then

    echo "Extracting bags"

    #create a new window called Bags-Extract in SIM session
    tmux new-window -t $SESSIONNAME -n Bags-Extract
    tmux send-keys -t $SESSIONNAME:Bags-Extract "source $UELAUNCHER_HOME/src/scripts/shared.sh; \
        eval $(parse_yaml $UELAUNCHER_HOME/config.yml)" C-m

    RUN_YOLO=$(yq e '.yolo.enable' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
    YOLO_FULL_COMMAND=""
    if [ $RUN_YOLO == true ]; then
        RUN_YOLO_DOCKER=$(yq e '.yolo.use_docker' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

        if [ $RUN_YOLO_DOCKER == true ]; then
            YOLO_FULL_COMMAND="docker kill $SESSIONNAME-yolo; \
            docker run -it --rm --gpus all --name $SESSIONNAME-yolo -v $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml:/config.yml -v $UELAUNCHER_HOME/bags/$SESSIONROOT/$CURRLIFE:/session -v $UELAUNCHER_HOME/src/scripts/yolo:/scripts ultralytics/ultralytics python3 /scripts/run_yolo.py;"
        else
            YOLO_FULL_COMMAND="python3 $UELAUNCHER_HOME/src/scripts/yolo/run_yolo.py $UELAUNCHER_HOME/bags/$SESSIONROOT/$CURRLIFE"
        fi
    fi

    tmux send-keys -t $SESSIONNAME:Bags-Extract \
        "python3 $UELAUNCHER_HOME/src/scripts/bag_extraction/listen_extraction_requests.py $EXTRACTION_PORT $SESSIONNAME $ROS_USE_DOCKER; \
        tmux kill-window -t $SESSIONNAME:ROS-Bags; \
        docker kill $SESSIONNAME-yolo; \
        $YOLO_FULL_COMMAND \
        python3 $UELAUNCHER_HOME/src/scripts/send_restart_signal.py $RESTART_SIGNAL_PORT; \
        tmux select-window -t $SESSIONNAME:Orchestrator; \
        tmux kill-window -t $SESSIONNAME:Bags-Extract; \
        " C-m

    init_extraction_argument=""
    if [ $simulation_target_is_time == false ]; then
        init_extraction_argument="$simulation_target"
    fi

    #execute rosbag info -y -k topics ros.bag > topics.yml in ROS-Bags
    if [ $ROS_USE_DOCKER == true ]; then
        tmux send-keys -t $SESSIONNAME:ROS-Bags "mv -f *.bag ros.bag; rosbag info -y -k topics ros.bag > topics.yml; python3 /scripts/bag_extraction/init_bag_extraction.py $init_extraction_argument $EXTRACTION_PORT" C-m
    else
        tmux send-keys -t $SESSIONNAME:ROS-Bags "mv -f *.bag ros.bag; rosbag info -y -k topics ros.bag > topics.yml; python3 $UELAUNCHER_HOME/src/scripts/bag_extraction/init_bag_extraction.py $init_extraction_argument $EXTRACTION_PORT $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml" C-m
    fi
else
    echo "Not extracting bags"
    tmux kill-window -t $SESSIONNAME:ROS-Bags
    python3 $UELAUNCHER_HOME/src/scripts/send_restart_signal.py $RESTART_SIGNAL_PORT
    tmux select-window -t $SESSIONNAME:Orchestrator
fi
