source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

if [ ! -d $unreal_project_path/Plugins ]; then
    mkdir $unreal_project_path/Plugins
else
    mv $unreal_project_path/Plugins $unreal_project_path/Plugins_original
    mkdir $unreal_project_path/Plugins
fi

#if the project is not a C++ project, convert it to C++
if [ ! -d $unreal_project_path/Source ]; then
    $UELAUNCHER_HOME/src/scripts/unreal/convert_project_to_cpp.sh
fi

uproject_path=$(dirname $unreal_project_path/*.uproject)
unreal_project_name=$(basename $unreal_project_path/*.uproject)
unreal_project_name=${unreal_project_name%.*}

docker run -ti --gpus all --net=host --pid=host -v $unreal_project_path/:/project -v $UELAUNCHER_HOME/src/plugins_link:/plugins_link -v $UELAUNCHER_HOME/src/plugins_copy:/plugins_copy -v $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/:/scripts/ --name $SESSIONNAME-unreal $unreal_docker_image bash --verbose -c "bash --verbose -c \"/scripts/start_game.sh\"; exec bash"