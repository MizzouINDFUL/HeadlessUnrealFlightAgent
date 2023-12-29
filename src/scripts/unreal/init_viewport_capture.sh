source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

tag_actors_command="tellunreal 'py unreal.VisionLib.tag_actors_from_string(unreal.EditorLevelLibrary.get_editor_world(), \"$unreal_actors_to_tag\")'"
add_vision_actor_command="tellunreal 'py unreal.VisionLib.add_vision_actor(unreal.EditorLevelLibrary.get_editor_world(), \"$simulation_tags_to_track\")'"

sleep 0.5
tmux send-keys -t $SESSIONNAME:tellunreal "$tag_actors_command" C-m
sleep 2
tmux send-keys -t $SESSIONNAME:tellunreal "$add_vision_actor_command" C-m
sleep 3.5
tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"game_instance_class\", unreal.SoftClassPath(\"/Script/ROSIntegration.ROSIntegrationGameInstance\")); unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"global_default_game_mode\", unreal.SoftClassPath(\"/Script/AirSim.AirSimGameMode\")); unreal.VisionLib.add_viewport_capture(unreal.EditorLevelLibrary.get_editor_world())'" C-m

# sleep 3
# tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"game_instance_class\", unreal.SoftClassPath(\"/Script/ROSIntegration.ROSIntegrationGameInstance\"))'" C-m
# sleep 3.5
# tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.GameMapsSettings.get_game_maps_settings().set_editor_property(\"global_default_game_mode\", unreal.SoftClassPath(\"/Script/AirSim.AirSimGameMode\")); unreal.VisionLib.add_viewport_capture(unreal.EditorLevelLibrary.get_editor_world())'" C-m