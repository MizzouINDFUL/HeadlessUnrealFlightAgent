source $UELAUNCHER_HOME/src/scripts/shared.sh
eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

if [ ! -d $unreal_project_path/Plugins ]; then
    mkdir $unreal_project_path/Plugins
else
    mv $unreal_project_path/Plugins $unreal_project_path/Plugins_original
    mkdir $unreal_project_path/Plugins
fi

#for each folder in $UELAUNCHER_HOME/src/plugins_link, link it to the project in $unreal_project_path/Plugins. force to replace existing links
for PLUGIN in $(ls $UELAUNCHER_HOME/src/plugins_link)
do
    if [ ! -L $unreal_project_path/Plugins/$PLUGIN ]; then
        ln -s $UELAUNCHER_HOME/src/plugins_link/$PLUGIN $unreal_project_path/Plugins/$PLUGIN
    else 
        rm $unreal_project_path/Plugins/$PLUGIN
        ln -s $UELAUNCHER_HOME/src/plugins_link/$PLUGIN $unreal_project_path/Plugins/$PLUGIN
    fi
done

#same, but cpoy plugins from plugins_copy instead of plugins_link and force to replace existing folders. use cp -r to copy the entire folder
for PLUGIN in $(ls $UELAUNCHER_HOME/src/plugins_copy)
do
    if [ ! -d $unreal_project_path/Plugins/$PLUGIN ]; then
        cp -r $UELAUNCHER_HOME/src/plugins_copy/$PLUGIN $unreal_project_path/Plugins/$PLUGIN
    else 
        yes | rm -r $unreal_project_path/Plugins/$PLUGIN
        cp -r $UELAUNCHER_HOME/src/plugins_copy/$PLUGIN $unreal_project_path/Plugins/$PLUGIN
    fi
done

#if the project is not a C++ project, convert it to C++
if [ ! -d $unreal_project_path/Source ]; then
    $UELAUNCHER_HOME/src/scripts/unreal/convert_project_to_cpp.sh
fi

uproject_path=$(dirname $unreal_project_path/*.uproject)
unreal_project_name=$(basename $unreal_project_path/*.uproject)
unreal_project_name=${unreal_project_name%.*}

#find uproject file
uproject=$(find $uproject_path -name "*.uproject")

echo $uproject

$unreal_engine_path/Engine/Build/BatchFiles/Linux/Build.sh $unreal_project_name"Editor" Linux DebugGame $uproject -IgnoreJunk -progress

EDITORNAME="UE4Editor-Linux-DegubGame"
#if UE4Editor doesn't exist, that means we are working with Unreal 5 and the file is called UnrealEditor
if [ ! -f $unreal_engine_path/Engine/Binaries/Linux/$EDITORNAME ]; then
    EDITORNAME="UnrealEditor-Linux-DebugGame"
fi

#launch the project
if [ $unreal_headless == true ]; then
    $unreal_engine_path/Engine/Binaries/Linux/$EDITORNAME "$uproject" -logcmds="LogDerivedDataCacheVerbose" -RenderOffscreen UNATTENDED
else
    $unreal_engine_path/Engine/Binaries/Linux/$EDITORNAME "$uproject" -logcmds="LogDerivedDataCacheVerbose" UNATTENDED
fi