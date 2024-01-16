source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

tag_actors_command="unreal.VisionLib.tag_actors_from_string(unreal.EditorLevelLibrary.get_editor_world(), \"$unreal_actors_to_tag\");"
add_vision_actor_command="unreal.VisionLib.add_vision_actor(unreal.EditorLevelLibrary.get_editor_world(), \"$simulation_tags_to_track\");"
set_game_instance_command="unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"game_instance_class\", unreal.SoftClassPath(\"/Script/ROSIntegration.ROSIntegrationGameInstance\")); unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"global_default_game_mode\", unreal.SoftClassPath(\"/Script/AirSim.AirSimGameMode\"));"
add_viewport_capture_command="unreal.VisionLib.add_viewport_capture(unreal.EditorLevelLibrary.get_editor_world());"
print_ready_command="print(\"VIEWPORT CAPTURE READY\")"

tellunreal_command="tellunreal 'py $tag_actors_command $add_vision_actor_command $set_game_instance_command $add_viewport_capture_command $print_ready_command'"

sleep 3.5
tmux send-keys -t $SESSIONNAME:tellunreal "$tellunreal_command" C-m