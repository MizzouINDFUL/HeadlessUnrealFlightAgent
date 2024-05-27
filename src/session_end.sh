#!/bin/bash
source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

#write "QUIT_EDITOR" to $UELAUNCHER_HOME/src/plugins_link/CommandLineExternal/command.txt and wait 0.5 seconds
echo "QUIT_EDITOR" > $UELAUNCHER_HOME/src/plugins_link/CommandLineExternal/command.txt
sleep 0.5
echo "" > $UELAUNCHER_HOME/src/plugins_link/CommandLineExternal/command.txt

docker kill $SESSIONNAME-ros; docker kill $SESSIONNAME-airsim-ros; docker kill $SESSIONNAME-yolo;

if [ $unreal_use_docker == true ]; then
    docker stop $SESSIONNAME-unreal
    docker rm $SESSIONNAME-unreal
fi

MORE_SESSIONS_EXIST=$(docker ps -a | grep $session_basename | wc -l)

UNREAL_PROJECT_PATH=$(yq e '.unreal.project_path' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

if [ ! $MORE_SESSIONS_EXIST -gt 1 ]; then
    if [ -d $UNREAL_PROJECT_PATH/Plugins_original ]; then
        if [ -d $UNREAL_PROJECT_PATH/Plugins ]; then
            yes | rm -r $UNREAL_PROJECT_PATH/Plugins
            mv $UNREAL_PROJECT_PATH/Plugins_original $UNREAL_PROJECT_PATH/Plugins
        fi
    fi
fi

SESSIONBASENAME=$(yq e '.session.basename' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

#check if the $UELAUNCHER_HOME/bags/$SESSIONROOT folder is empty. If it is, delete it
if [ ! "$(ls -A $UELAUNCHER_HOME/bags/$SESSIONBASENAME)" ]; then
    rm -r $UELAUNCHER_HOME/bags/$SESSIONBASENAME
    
    if [ -f $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log ]; then
        rm $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log
    fi
else
    if [ -f $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log ]; then
        mv $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log $UELAUNCHER_HOME/bags/$SESSIONBASENAME/$SESSIONNAME-Unreal.log
    fi
    cp $UELAUNCHER_HOME/src/scripts/yolo/generate_videos.py $UELAUNCHER_HOME/bags/$SESSIONBASENAME/generate_videos.py
fi

rm $UELAUNCHER_HOME/src/logs/$SIM_START_DATE-Unreal.log
rm $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml