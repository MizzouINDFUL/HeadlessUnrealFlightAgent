UNREAL_START_GAME=$(yq e '.unreal.start_game' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
USE_MRQ=$(yq e '.unreal.use_mrq' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
TELLUNREAL_PORT=$(yq e '.ports_to_reserve[0].tellunreal_listener' tmp/$SESSIONNAME-config.yml)

if [ $UNREAL_START_GAME == true ]; then
    sleep 15;
    
    if [ $USE_MRQ == true ]; then
        python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT "py run_mrq.py"
    else
        python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT "py unreal.MindfulLib.start_life()"
    fi
fi