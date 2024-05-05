source $UELAUNCHER_HOME/src/scripts/shared.sh

LIFE_NUM=$(yq e '.current_life' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
MAX_LIFE_NUM=$(yq e '.session.max_lives' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

docker run -it --rm --net host -e DISPLAY=$DISPLAY -v $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml:/config.yml -v ./src/airsim-ros/shared:/root/shared -v $UELAUNCHER_HOME/bags/$SESSIONROOT:/root/session/ --name $SESSIONNAME-airsim-ros unreal-launcher-airsim-ros bash -c "~/shared/start_tmux_windows.sh $LIFE_NUM $MAX_LIFE_NUM"
