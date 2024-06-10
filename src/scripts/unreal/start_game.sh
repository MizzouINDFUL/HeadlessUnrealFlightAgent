UNREAL_START_GAME=$(yq e '.unreal.start_game' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

if [ $UNREAL_START_GAME == true ]; then
    sleep 15;
    tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py run_mrq.py'" C-m
fi