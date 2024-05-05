source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

RESTART_SIGNAL_PORT=$(yq e '.ports_to_reserve[1].life_restart_listener' tmp/$SESSIONNAME-config.yml)
python3 $UELAUNCHER_HOME/src/scripts/listen_restart_signal.py $RESTART_SIGNAL_PORT

CURR_LIFE=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
MAX_LIVES=$(yq e '.session.end_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

if [ $CURR_LIFE -ge $MAX_LIVES ]; then
    echo "Simulation has ended"
    tmux kill-session -t $SESSIONNAME
else
    echo "Simulation is restarting"
    tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py on_life_ended.py $CURR_LIFE $simulation_num_lives'" C-m
    sleep 3;

    CURR_LIFE=$(($CURR_LIFE+1))
    yq e ".current_life = $CURR_LIFE" -i $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml

    tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py run_mrq.py $CURR_LIFE'" C-m
    bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/start_airsim_rosbag.sh
    bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/listen_restart_signal.sh
    bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/unreal/post_start_game.sh
    bind_script_to_event "MRQ SIM FINISHED" $UELAUNCHER_HOME/src/scripts/unreal/mrq_done.sh true
    # bind_script_to_event "Bringing up level for play took" $UELAUNCHER_HOME/src/scripts/unreal/stop_game.sh true
fi

sleep 8