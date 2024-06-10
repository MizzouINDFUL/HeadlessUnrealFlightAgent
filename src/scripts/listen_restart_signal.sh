source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

RESTART_SIGNAL_PORT=$(yq e '.ports_to_reserve[1].life_restart_listener' tmp/$SESSIONNAME-config.yml)
python3 $UELAUNCHER_HOME/src/scripts/listen_restart_signal.py $RESTART_SIGNAL_PORT

CURR_LIFE=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
MAX_LIVES=$(yq e '.session.end_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

if [ $CURR_LIFE -ge $MAX_LIVES ] && [ $MAX_LIVES != -1 ]; then
    echo "Simulation has ended"
    tmux kill-session -t $SESSIONNAME
else
    echo "Simulation is restarting"
    TELLUNREAL_PORT=$(yq e '.ports_to_reserve[0].tellunreal_listener' tmp/$SESSIONNAME-config.yml)
    python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT "py on_life_ended.py $CURR_LIFE $simulation_num_lives"
    sleep 3;

    CURR_LIFE=$(($CURR_LIFE+1))
    yq e ".current_life = $CURR_LIFE" -i $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml

    # tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py run_mrq.py $CURR_LIFE'" C-m
    tellunreal_command="py print('BEGIN EXPERIMENT')"
    python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT "$tellunreal_command"

    TOTAL_CALLS=$(python3 src/scripts/get_num_bindings.py $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

    echo "Found $TOTAL_CALLS scripts to bind in the config file"

    for ((i=0; i<$TOTAL_CALLS; i++))
    do
        event=$(python3 src/scripts/get_call_args.py $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml $i 0)
        script=$(python3 src/scripts/get_call_args.py $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml $i 1)
        clear_logs=$(python3 src/scripts/get_call_args.py $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml $i 2)
        should_bind=$(python3 src/scripts/get_call_args.py $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml $i 3)

        if [ "$should_bind" == true ]; then
            bind_script_to_event "$event" $script $clear_logs
        fi
    done

fi

sleep 8