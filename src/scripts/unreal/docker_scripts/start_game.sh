#if ~/Documents/AirSim doesn't exist, create it
if [ ! -d ~/Documents/AirSim ]; then
    mkdir ~/Documents
    mkdir ~/Documents/AirSim
fi

#copy /scripts/settings.json to ~/Documents/AirSim/settings.json
cp /scripts/settings.json ~/Documents/AirSim/settings.json

for PLUGIN in $(ls /plugins_link)
do
    if [ ! -L /project/Plugins/$PLUGIN ]; then
        ln -s /plugins_link/$PLUGIN /project/Plugins/$PLUGIN
    else 
        rm /project/Plugins/$PLUGIN
        ln -s /plugins_link/$PLUGIN /project/Plugins/$PLUGIN
    fi
done

#same, but cpoy plugins from plugins_copy instead of plugins_link and force to replace existing folders. use cp -r to copy the entire folder
for PLUGIN in $(ls /plugins_copy)
do
    if [ ! -d /project/Plugins/$PLUGIN ]; then
        cp -r /plugins_copy/$PLUGIN /project/Plugins/$PLUGIN
    else 
        yes | rm -r /project/Plugins/$PLUGIN
        cp -r /plugins_copy/$PLUGIN /project/Plugins/$PLUGIN
    fi
done

unreal_project_name=$(basename /project/*.uproject)
unreal_project_name=${unreal_project_name%.*}

/home/ue4/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh $unreal_project_name"Editor" Linux DebugGame /project/$unreal_project_name.uproject -IgnoreJunk -progress
/home/ue4/UnrealEngine/Engine/Binaries/Linux/UnrealEditor-Linux-DebugGame /project/$unreal_project_name.uproject -RenderOffscreen -imgOutputX=$1 -imgOutputY=$2 -ground_truth=$3 -num_frames=$4
