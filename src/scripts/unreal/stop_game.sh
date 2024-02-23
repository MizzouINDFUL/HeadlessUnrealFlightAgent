source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

CURRLIFE=$(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)

echo "stop_game script entrypint"

if [ $simulation_target_is_time == true ]; then
    echo "Ending life $CURRLIFE in $simulation_target seconds"
    sleep $simulation_target
else
    echo "Ending life $CURRLIFE after $simulation_target messages"
    while :
    do
        msg_count=$(docker exec -it $SESSIONNAME-ros /bin/bash -c "source /opt/ros/noetic/setup.bash; rostopic echo -n 1 /unreal_ros/message_count")
        echo "looking at $msg_count messages in /unreal_ros/"
        if [ -n "$msg_count" ]; then
            msg_count=$(echo $msg_count | tr -d '\n')
            msg_count=$(echo $msg_count | sed 's/[^0-9]//g')
            echo "msg_count is $msg_count"
            if [ $msg_count -ge $simulation_target ]; then
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

#send 'py unreal.MindfulLib.stop_life()' to tellunreal window in SIM session
tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.MindfulLib.stop_life()'" C-m

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
        docker run -it --rm --gpus all --name $SESSIONNAME-yolo -v $UELAUNCHER_HOME/bags/$SIM_START_DATE/$(($CURRLIFE+1)):/session -v $UELAUNCHER_HOME/src/scripts/yolo:/scripts ultralytics/ultralytics python3 /scripts/run_yolo.py; \
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
