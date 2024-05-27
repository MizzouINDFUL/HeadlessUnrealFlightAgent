source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

#temp

set_game_instance_command="unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"game_instance_class\", unreal.SoftClassPath(\"/Script/ROSIntegration.ROSIntegrationGameInstance\")); unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"global_default_game_mode\", unreal.SoftClassPath(\"/Script/AirSim.AirSimGameMode\"));"
add_viewport_capture_command="unreal.VisionLib.add_viewport_capture(unreal.EditorLevelLibrary.get_editor_world());"
print_ready_command="print(\"VIEWPORT CAPTURE READY\")"

tellunreal_command="tellunreal 'py $set_game_instance_command $print_ready_command'"
sleep 1
tmux send-keys -t $SESSIONNAME:tellunreal "$tellunreal_command" C-m
