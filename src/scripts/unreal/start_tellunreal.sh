TELLUNREAL_PORT=$(yq e '.ports_to_reserve[0].tellunreal_listener' tmp/$SESSIONNAME-config.yml)

tmux send-keys -t $SESSIONNAME:tellunreal "alias tellunreal='function _tellunreal(){ python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT \"\$@\"; };_tellunreal'" Enter

src/scripts/unreal/init_viewport_capture.sh