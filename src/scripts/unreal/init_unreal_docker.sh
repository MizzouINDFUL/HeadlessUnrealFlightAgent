source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

SKIP_PLUGINS_INSTALL=$(yq e '.session.skip_plugins_install' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
PLUGINS_VOLUMES="-v $UELAUNCHER_HOME/src/plugins_link:/plugins_link -v $UELAUNCHER_HOME/src/plugins_copy:/plugins_copy"

if [ ! -d $unreal_project_path/Plugins ]; then
        mkdir $unreal_project_path/Plugins
fi

if [ ! -d $unreal_project_path/Source ]; then
        $UELAUNCHER_HOME/src/scripts/unreal/convert_project_to_cpp.sh
fi

if [ "$SKIP_PLUGINS_INSTALL" = false ]; then
    PLUGINS_VOLUMES=""

    if [ ! -d $unreal_project_path/Plugins ]; then
        mkdir $unreal_project_path/Plugins
    else
        cp -r $unreal_project_path/Plugins $unreal_project_path/Plugins_original
    fi
fi

uproject_path=$(dirname $unreal_project_path/*.uproject)
unreal_project_name=$(basename $unreal_project_path/*.uproject)
unreal_project_name=${unreal_project_name%.*}

echo "Full docker command: docker run -ti --gpus all --net=host --pid=host -v $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml:/config.yml -v $unreal_project_path/$SESSIONNAME-cmd.txt:/cmd.txt -v $unreal_project_path/:/project $PLUGINS_VOLUMES -v $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/:/scripts/ --name $SESSIONNAME-unreal $unreal_docker_image bash --verbose -c \"bash --verbose -c \"/scripts/start_game.sh\" \""

docker stop $SESSIONNAME-unreal; docker rm $SESSIONNAME-unreal
docker run -ti --gpus all --net=host --pid=host -v $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml:/config.yml -v $unreal_project_path/$SESSIONNAME-cmd.txt:/cmd.txt -v $unreal_project_path/:/project $PLUGINS_VOLUMES -v $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/:/scripts/ --name $SESSIONNAME-unreal $unreal_docker_image bash --verbose -c "bash --verbose -c \"/scripts/start_game.sh\" "
