#!/bin/bash
source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

docker kill $SESSIONNAME-ros; docker kill $SESSIONNAME-airsim-ros; docker kill $SESSIONNAME-yolo;

if [ $unreal_use_docker == true ]; then
    if [ $unreal_separate_session == false ]; then
        docker stop $SESSIONNAME-unreal
        docker rm $SESSIONNAME-unreal
    fi
fi

MORE_SESSIONS_EXIST=$(docker ps -a | grep $session_basename | wc -l)

if [ ! $MORE_SESSIONS_EXIST -gt 1 ]; then
    if [ -d $unreal_project_path/Plugins ]; then
        yes | rm -r $unreal_project_path/Plugins
        mv $unreal_project_path/Plugins_original $unreal_project_path/Plugins
    fi
fi

#check if the $UELAUNCHER_HOME/bags/$SIM_START_DATE folder is empty. If it is, delete it
if [ ! "$(ls -A $UELAUNCHER_HOME/bags/$SIM_START_DATE)" ]; then
    rm -r $UELAUNCHER_HOME/bags/$SIM_START_DATE
fi