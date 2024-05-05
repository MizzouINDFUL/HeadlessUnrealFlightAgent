source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

SKIP_PLUGINS_INSTALL=$(yq e '.session.skip_plugin_install' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
UNREAL_PROJECT_PATH=$(yq e '.unreal.project_path' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
CUSTOM_UE_USER=$(yq e '.unreal.user' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
UE_COMMAND_PREFIX=""

if [ "$CUSTOM_UE_USER" != "" ]; then
    UE_COMMAND_PREFIX="sudo -u $CUSTOM_UE_USER"
fi

if [ ! -d $UNREAL_PROJECT_PATH/Plugins ]; then
    mkdir $UNREAL_PROJECT_PATH/Plugins
else
    cp -r $UNREAL_PROJECT_PATH/Plugins $UNREAL_PROJECT_PATH/Plugins_original
fi

if [ "$SKIP_PLUGINS_INSTALL" = false ]; then
    #for each folder in $UELAUNCHER_HOME/src/plugins_link, link it to the project in $UNREAL_PROJECT_PATH/Plugins. force to replace existing links
    for PLUGIN in $(ls $UELAUNCHER_HOME/src/plugins_link)
    do
        echo "Linking $PLUGIN"
        if [ ! -L $UNREAL_PROJECT_PATH/Plugins/$PLUGIN ]; then
            ln -s $UELAUNCHER_HOME/src/plugins_link/$PLUGIN $UNREAL_PROJECT_PATH/Plugins/$PLUGIN
        else 
            rm $UNREAL_PROJECT_PATH/Plugins/$PLUGIN
            ln -s $UELAUNCHER_HOME/src/plugins_link/$PLUGIN $UNREAL_PROJECT_PATH/Plugins/$PLUGIN
        fi
    done

    #same, but cpoy plugins from plugins_copy instead of plugins_link and force to replace existing folders. use cp -r to copy the entire folder
    for PLUGIN in $(ls $UELAUNCHER_HOME/src/plugins_copy)
    do
        echo "Copying $PLUGIN"
        if [ ! -d $UNREAL_PROJECT_PATH/Plugins/$PLUGIN ]; then
            cp -r $UELAUNCHER_HOME/src/plugins_copy/$PLUGIN $UNREAL_PROJECT_PATH/Plugins/$PLUGIN
        else 
            yes | rm -r $UNREAL_PROJECT_PATH/Plugins/$PLUGIN
            cp -r $UELAUNCHER_HOME/src/plugins_copy/$PLUGIN $UNREAL_PROJECT_PATH/Plugins/$PLUGIN
        fi
    done
else
    echo "Skipping plugins installation"
fi

#if the project is not a C++ project, convert it to C++
if [ ! -d $UNREAL_PROJECT_PATH/Source ]; then
    $UELAUNCHER_HOME/src/scripts/unreal/convert_project_to_cpp.sh
fi

uproject_path=$(dirname $UNREAL_PROJECT_PATH/*.uproject)
unreal_project_name=$(basename $UNREAL_PROJECT_PATH/*.uproject)
unreal_project_name=${unreal_project_name%.*}

#find uproject file
uproject=$(find $uproject_path -name "*.uproject")

$unreal_local_path/Engine/Build/BatchFiles/Linux/Build.sh $unreal_project_name"Editor" Linux DebugGame $uproject -IgnoreJunk -progress

EDITORNAME="UE4Editor-Linux-DegubGame"
#if UE4Editor doesn't exist, that means we are working with Unreal 5 and the file is called UnrealEditor
if [ ! -f $unreal_local_path/Engine/Binaries/Linux/$EDITORNAME ]; then
    EDITORNAME="UnrealEditor-Linux-DebugGame"
fi

#in the project directory, there is a $SESSIONNAME-cmd.txt file that contains the arguments to be passed to 
args=$(< $UNREAL_PROJECT_PATH/$SESSIONNAME-cmd.txt)
$UE_COMMAND_PREFIX $unreal_local_path/Engine/Binaries/Linux/$EDITORNAME "$uproject" $args
