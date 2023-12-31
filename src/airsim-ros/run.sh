source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

LIFE_NUM=$(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)
LIFE_NUM=$(($LIFE_NUM+1))

docker run -it --rm --net host -e DISPLAY=$DISPLAY -v ./src/airsim-ros/shared:/root/shared --name $SESSIONNAME-airsim-ros unreal-launcher-airsim-ros bash -c "~/shared/start_tmux_windows.sh $LIFE_NUM"
