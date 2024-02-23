#!/bin/bash
source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

#write "QUIT_EDITOR" to $UELAUNCHER_HOME/src/plugins_link/CommandLineExternal/command.txt and wait 0.5 seconds
echo "QUIT_EDITOR" > $UELAUNCHER_HOME/src/plugins_link/CommandLineExternal/command.txt
sleep 0.5
echo "" > $UELAUNCHER_HOME/src/plugins_link/CommandLineExternal/command.txt

docker kill $SESSIONNAME-ros; docker kill $SESSIONNAME-airsim-ros; docker kill $SESSIONNAME-yolo;

if [ $unreal_use_docker == true ]; then
    if [ $unreal_separate_session == false ]; then
        docker stop $SESSIONNAME-unreal
        docker rm $SESSIONNAME-unreal
    fi
fi

MORE_SESSIONS_EXIST=$(docker ps -a | grep $session_basename | wc -l)

if [ ! $MORE_SESSIONS_EXIST -gt 1 ]; then
    if [ -d $unreal_project_path/Plugins_original ]; then
        if [ -d $unreal_project_path/Plugins ]; then
            yes | rm -r $unreal_project_path/Plugins
            mv $unreal_project_path/Plugins_original $unreal_project_path/Plugins
        fi
    fi
fi

#check if the $UELAUNCHER_HOME/bags/$SIM_START_DATE folder is empty. If it is, delete it
if [ ! "$(ls -A $UELAUNCHER_HOME/bags/$SIM_START_DATE)" ]; then
    rm -r $UELAUNCHER_HOME/bags/$SIM_START_DATE
    
    if [ -f $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log ]; then
        rm $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log
    fi
else
    if [ -f $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log ]; then
        mv $UELAUNCHER_HOME/tmp/$SIM_START_DATE-Unreal.log $UELAUNCHER_HOME/bags/$SIM_START_DATE/Unreal.log
    fi
    cp $UELAUNCHER_HOME/src/scripts/yolo/generate_videos.py $UELAUNCHER_HOME/bags/$SIM_START_DATE/generate_videos.py
fi