CURR_LIFE=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
TELLUNREAL_PORT=$(yq e '.ports_to_reserve[0].tellunreal_listener' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

tellunreal_command="py level_load.py $CURR_LIFE"

python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT "$tellunreal_command"