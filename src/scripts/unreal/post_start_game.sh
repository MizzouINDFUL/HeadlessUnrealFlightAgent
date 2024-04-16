source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

# tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.VisionLib.add_data_grabber(unreal.UnrealEditorSubsystem().get_game_world(), \"SimpleFlight\" ,\"$simulation_tags_to_track\")'" C-m