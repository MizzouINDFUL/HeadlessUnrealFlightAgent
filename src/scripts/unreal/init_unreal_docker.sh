source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

SKIP_PLUGINS_INSTALL=$(yq e '.session.skip_plugin_install' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
PLUGINS_VOLUMES="-v $UELAUNCHER_HOME/src/plugins_link:/plugins_link -v $UELAUNCHER_HOME/src/plugins_copy:/plugins_copy"
UNREAL_PROJECT_PATH=$(yq e '.unreal.project_path' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
UNREAL_DOCKER_IMAGE=$(yq e '.unreal.docker_image' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

if [ ! -d $UNREAL_PROJECT_PATH/Plugins ]; then
        mkdir $UNREAL_PROJECT_PATH/Plugins
fi

if [ ! -d $UNREAL_PROJECT_PATH/Source ]; then
        $UELAUNCHER_HOME/src/scripts/unreal/convert_project_to_cpp.sh
fi

if [ "$SKIP_PLUGINS_INSTALL" = false ]; then
    PLUGINS_VOLUMES=""

    if [ ! -d $UNREAL_PROJECT_PATH/Plugins ]; then
        mkdir $UNREAL_PROJECT_PATH/Plugins
    else
        cp -r $UNREAL_PROJECT_PATH/Plugins $UNREAL_PROJECT_PATH/Plugins_original
    fi
fi

uproject_path=$(dirname $UNREAL_PROJECT_PATH/*.uproject)
unreal_project_name=$(basename $UNREAL_PROJECT_PATH/*.uproject)
unreal_project_name=${unreal_project_name%.*}

echo "Full docker command: docker run -ti --gpus all --net=host --pid=host -v $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml:/config.yml -v $UNREAL_PROJECT_PATH/$SESSIONNAME-cmd.txt:/cmd.txt -v $UNREAL_PROJECT_PATH/:/project $PLUGINS_VOLUMES -v $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/:/scripts/ --name $SESSIONNAME-unreal $UNREAL_DOCKER_IMAGE bash --verbose -c \"bash --verbose -c \"/scripts/start_game.sh\" \""

docker stop $SESSIONNAME-unreal; docker rm $SESSIONNAME-unreal
docker run -ti --gpus all --net=host --pid=host -v $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml:/config.yml -v $UNREAL_PROJECT_PATH/$SESSIONNAME-cmd.txt:/cmd.txt -v $UNREAL_PROJECT_PATH/:/project $PLUGINS_VOLUMES -v $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/:/scripts/ --name $SESSIONNAME-unreal $UNREAL_DOCKER_IMAGE bash --verbose -c "bash --verbose -c \"/scripts/start_game.sh\" "
