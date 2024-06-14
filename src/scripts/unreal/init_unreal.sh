source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

SKIP_PLUGINS_INSTALL=$(yq e '.session.skip_plugin_install' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
UNREAL_PROJECT_PATH=$(yq e '.unreal.project_path' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
CUSTOM_UE_USER=$(yq e '.unreal.user' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)
SETTINGS_JSON_PATH="$UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/settings.json"
UE_COMMAND_PREFIX=""

#if ~/Documents/AirSim doesn't exist, create it
if [ ! -d ~/Documents/AirSim ]; then
    if [ ! -d ~/Documents ]; then
        mkdir ~/Documents
    fi
    mkdir ~/Documents/AirSim
fi

if [ "$CUSTOM_UE_USER" != "" ]; then
    if [ ! -d /home/$CUSTOM_UE_USER ]; then
        mkdir /home/$CUSTOM_UE_USER
    fi
    if [ ! -d /home/$CUSTOM_UE_USER/Documents ]; then
            mkdir /home/$CUSTOM_UE_USER/Documents
    fi
    if [ ! -d /home/$CUSTOM_UE_USER/Documents/AirSim ]; then
        mkdir /home/$CUSTOM_UE_USER/Documents/AirSim
    fi
fi

# cp $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/settings.json ~/Documents/AirSim/settings.json
cp $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/settings.json $UELAUNCHER_HOME/tmp/$SESSIONNAME-airsim-settings.json
chmod a+x ~/Documents/AirSim/settings.json
chmod a+r ~/Documents/AirSim/settings.json
python3 $UELAUNCHER_HOME/src/scripts/unreal/docker_scripts/set_airsim_port.py "$UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml" "$UELAUNCHER_HOME/tmp/$SESSIONNAME-airsim-settings.json"
mv $UELAUNCHER_HOME/tmp/$SESSIONNAME-airsim-settings.json ~/Documents/AirSim/settings.json
if [ "$CUSTOM_UE_USER" != "" ]; then
    cp ~/Documents/AirSim/settings.json /home/$CUSTOM_UE_USER/Documents/AirSim/settings.json
fi

echo "AirSim Settings:"
cat ~/Documents/AirSim/settings.json

if [ "$CUSTOM_UE_USER" != "" ]; then
    UE_COMMAND_PREFIX="sudo -u $CUSTOM_UE_USER"
fi

if [ ! -d $UNREAL_PROJECT_PATH/Plugins ]; then
    mkdir $UNREAL_PROJECT_PATH/Plugins
else
    mv $UNREAL_PROJECT_PATH/Plugins $UNREAL_PROJECT_PATH/Plugins_original
fi

if [ $SKIP_PLUGINS_INSTALL != true ]; then

    #ensure that Plugins folder exists
    if [ ! -d $UNREAL_PROJECT_PATH/Plugins ]; then
        mkdir $UNREAL_PROJECT_PATH/Plugins
    fi

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

    #same, but copy plugins from plugins_copy instead of plugins_link and force to replace existing folders. use cp -r to copy the entire folder
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

UNREAL_LOCAL_PATH=$(yq e '.unreal.local_path' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)

$UNREAL_LOCAL_PATH/Engine/Build/BatchFiles/Linux/Build.sh $unreal_project_name"Editor" Linux DebugGame $uproject -IgnoreJunk -progress

EDITORNAME="UE4Editor-Linux-DegubGame"
#if UE4Editor doesn't exist, that means we are working with Unreal 5 and the file is called UnrealEditor
if [ ! -f $UNREAL_LOCAL_PATH/Engine/Binaries/Linux/$EDITORNAME ]; then
    EDITORNAME="UnrealEditor-Linux-DebugGame"
fi

#Dockerfile edge case
if [ -d "/home/ue4" ]; then
    mkdir /home/ue4/.config
    chmod a+x /home/ue4/.config
    chmod a+w /home/ue4/.config
    chmod 777 /home/ue4/UnrealEngine/Engine
    chmod 777 $UNREAL_PROJECT_PATH/*
fi

#in the project directory, there is a $SESSIONNAME-cmd.txt file that contains the arguments to be passed to 
args=$(< $UNREAL_PROJECT_PATH/$SESSIONNAME-cmd.txt)

#append config path to args
# -sim_config
args="$args -sim_config=$UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml"

echo "Running $EDITORNAME $uproject $args"
$UE_COMMAND_PREFIX $UNREAL_LOCAL_PATH/Engine/Binaries/Linux/$EDITORNAME "$uproject" $args
