source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

python3 $UELAUNCHER_HOME/src/scripts/listen_restart_signal.py

CURR_LIFE=$(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)

if [ $CURR_LIFE -ge $simulation_num_lives ]; then
    echo "Simulation has ended"
    tmux kill-session -t $SESSIONNAME
else
    echo "Simulation is restarting"
    tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py on_life_ended.py $CURR_LIFE $simulation_num_lives'" C-m
    sleep 3;
    # tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.MindfulLib.start_life()'" C-m
    tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py run_mrq.py $CURR_LIFE'" C-m
    bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/start_airsim_rosbag.sh
    bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/listen_restart_signal.sh
    bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/unreal/post_start_game.sh
    bind_script_to_event "MRQ SIM FINISHED" $UELAUNCHER_HOME/src/scripts/unreal/mrq_done.sh true
    # bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/unreal/stop_game.sh true
fi 

sleep 8