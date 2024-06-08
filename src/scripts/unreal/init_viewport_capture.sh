source $UELAUNCHER_HOME/src/scripts/shared.sh

TELLUNREAL_PORT=$(yq e '.ports_to_reserve[0].tellunreal_listener' tmp/$SESSIONNAME-config.yml)

set_game_instance_command="unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"game_instance_class\", unreal.SoftClassPath(\"/Script/ROSIntegration.ROSIntegrationGameInstance\")); unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"global_default_game_mode\", unreal.SoftClassPath(\"/Script/AirSim.AirSimGameMode\"));"
add_viewport_capture_command="unreal.VisionLib.add_viewport_capture(unreal.EditorLevelLibrary.get_editor_world());"
print_ready_command="print(\"BEGIN EXPERIMENT\")"

tellunreal_command="tellunreal 'py $set_game_instance_command $print_ready_command'"
sleep 1
# tmux send-keys -t $SESSIONNAME:tellunreal "$tellunreal_command" C-m
python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT "py $set_game_instance_command $print_ready_command"
