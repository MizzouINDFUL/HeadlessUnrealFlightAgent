source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

# previously we set life to an intermediate yml life: yq e '.current_life = 0' -i $HOME_DIR/tmp/$SESSIONNAME-config.yml
CURRLIFE=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
SIMULATION_TARGET_IS_TIME=$(yq e '.session.target_is_time' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
SIMULATION_TARGET=$(yq e '.session.target' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

echo "stop_game script entrypint"

if [ $SIMULATION_TARGET_IS_TIME == true ]; then
    echo "Ending life $CURRLIFE in $SIMULATION_TARGET seconds"
    sleep $SIMULATION_TARGET
else
    echo "Ending life $CURRLIFE after $SIMULATION_TARGET messages"
    while :
    do
        msg_count=$(docker exec -it $SESSIONNAME-airsim-ros /bin/bash -c "source /opt/ros/noetic/setup.bash; rostopic echo -n 1 /unreal_ros/message_count")
        echo "looking at $msg_count messages in /unreal_ros/"
        if [ -n "$msg_count" ]; then
            msg_count=$(echo $msg_count | tr -d '\n')
            msg_count=$(echo $msg_count | sed 's/[^0-9]//g')
            echo "msg_count is $msg_count"
            if [ $msg_count -ge $SIMULATION_TARGET ]; then
                break
            fi
        else
            echo "msg_count is empty"
        fi
        sleep 5
    done
fi

echo "Ending life $CURRLIFE"

#send Ctrl+C to ROS-Bags in SIM session
tmux send-keys -t $SESSIONNAME:ROS-Bags C-c

#kill the AirSim window in SIM
docker kill unreal-launcher-airsim-ros
tmux kill-window -t $SESSIONNAME:AirSim

sleep 5

tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.MindfulLib.stop_life()'" C-m

RESTART_SIGNAL_PORT=$(yq e '.ports_to_reserve[1].life_restart_listener' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
EXTRACTION_PORT=$(yq e '.ports_to_reserve[2].rosbag_extraction_listener' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

echo "Listening for restart signal on port $RESTART_SIGNAL_PORT"
echo "Listening for extraction requests on port $EXTRACTION_PORT"

if [ $simulation_extract_on_end == true ]; then

    echo "Extracting bags"

    #create a new window called Bags-Extract in SIM session
    tmux new-window -t $SESSIONNAME -n Bags-Extract
    tmux send-keys -t $SESSIONNAME:Bags-Extract "source $UELAUNCHER_HOME/src/scripts/shared.sh" C-m

    tmux send-keys -t $SESSIONNAME:Bags-Extract \
        "python3 $UELAUNCHER_HOME/src/scripts/bag_extraction/listen_extraction_requests.py $EXTRACTION_PORT; \
        tmux kill-window -t $SESSIONNAME:ROS-Bags; \
        docker kill $SESSIONNAME-yolo; \
        docker run -it --rm --gpus all --name $SESSIONNAME-yolo -v $UELAUNCHER_HOME/bags/$SESSIONROOT/$(($CURRLIFE+1)):/session -v $UELAUNCHER_HOME/src/scripts/yolo:/scripts ultralytics/ultralytics python3 /scripts/run_yolo.py; \
        export NUM_LIVES=$(($NUM_LIVES+1)); \
        python3 $UELAUNCHER_HOME/src/scripts/send_restart_signal.py $RESTART_SIGNAL_PORT; \
        sleep 50; \
        tmux select-window -t $SESSIONNAME:Orchestrator; \
        tmux kill-window -t $SESSIONNAME:Bags-Extract; \
        " C-m

    init_extraction_argument=""
    if [ $SIMULATION_TARGET_IS_TIME == false ]; then
        init_extraction_argument="$SIMULATION_TARGET"
    fi

    #execute rosbag info -y -k topics ros.bag > topics.yml in ROS-Bags
    tmux send-keys -t $SESSIONNAME:ROS-Bags "mv *.bag ros.bag; rosbag info -y -k topics ros.bag > topics.yml; python3 /scripts/bag_extraction/init_bag_extraction.py $init_extraction_argument $EXTRACTION_PORT" C-m
else
    echo "Not extracting bags"
    tmux kill-window -t $SESSIONNAME:ROS-Bags
    export NUM_LIVES=$(($NUM_LIVES+1))
    python3 $UELAUNCHER_HOME/src/scripts/send_restart_signal.py $RESTART_SIGNAL_PORT
    tmux select-window -t $SESSIONNAME:Orchestrator
fi
