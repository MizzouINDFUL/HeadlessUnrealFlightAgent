#if ~/Documents/AirSim doesn't exist, create it
if [ ! -d ~/Documents/AirSim ]; then
    mkdir ~/Documents
    mkdir ~/Documents/AirSim
fi

# cp /scripts/settings.json ~/Documents/AirSim/settings.json
mkdir /tmp
cp /scripts/settings.json /tmp/settings.json

#a python one liner that reads the port value from a yaml /config.yml (ports_to_reserve: - airsim_api: <PORT>) and sets it to the settings.json file(ApiServerPort)
python3 /scripts/set_airsim_port.py "/config.yml" "/tmp/settings.json"
cp /tmp/settings.json ~/Documents/AirSim/settings.json

cat ~/Documents/AirSim/settings.json

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

args=$(< /cmd.txt)

#append config path to the arguments
# -sim_config
args="$args -sim_config=/config.yml"

/home/ue4/UnrealEngine/Engine/Build/BatchFiles/Linux/Build.sh $unreal_project_name"Editor" Linux DebugGame /project/$unreal_project_name.uproject -IgnoreJunk -progress
/home/ue4/UnrealEngine/Engine/Binaries/Linux/UnrealEditor-Linux-DebugGame /project/$unreal_project_name.uproject $args