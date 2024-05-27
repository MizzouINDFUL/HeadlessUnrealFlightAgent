#this will convert the unreal project from BP only to C++ and BP by adding a source folder, adding a project module, and Editor Target file

source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

UNREAL_PROJECT_PATH=$(yq e '.unreal.project_path' $UELAUNCHER_HOME/tmp/$SESSIONNAME-config.yml)


mkdir $UNREAL_PROJECT_PATH/Source
project_name=$(basename $UNREAL_PROJECT_PATH/*.uproject)
project_name=${project_name%.*}
mkdir $UNREAL_PROJECT_PATH/Source/$project_name
mkdir $UNREAL_PROJECT_PATH/Source/$project_name/Public
mkdir $UNREAL_PROJECT_PATH/Source/$project_name/Private

#create [ProjectName]Editor.Target.cs file

editortarget=$project_name"EditorTarget"
echo "using UnrealBuildTool;
using System.Collections.Generic;

public class $editortarget : TargetRules
{
        public $editortarget(TargetInfo Target) : base(Target)
    {
        DefaultBuildSettings = BuildSettingsVersion.V2;
        Type = TargetType.Editor;
        ExtraModuleNames.AddRange(new string[] { \"$project_name\" });
    }
}" > $UNREAL_PROJECT_PATH/Source/$project_name"Editor.Target.cs"

#create [ProjectName].Build.cs file

buildcs=$project_name".Build.cs"
echo "using UnrealBuildTool;

public class $project_name : ModuleRules
{
        public $project_name(ReadOnlyTargetRules Target) : base(Target)
    {
        /* AIModule is not assigned by default, must be added when dealing with AI in your project */
        PublicDependencyModuleNames.AddRange(new string[] {
            \"Core\",
            \"CoreUObject\",
            \"Engine\",
            \"InputCore\",
            \"AIModule\",
            /* Temporarily added GameplayTasks to workaround 4.12 compilation bug. */
            \"GameplayTasks\",
            \"NavigationSystem\",
        });

        PrivateDependencyModuleNames.AddRange(new string[] {  });
        }
}" > $UNREAL_PROJECT_PATH/Source/$project_name/$buildcs

#create [ProjectName].h

echo "#pragma once

#include \"CoreMinimal.h\"
#include \"Engine/Engine.h\"" > $UNREAL_PROJECT_PATH/Source/$project_name/$project_name".h"

#create [ProjectName].cpp

echo "#include \"$project_name.h\"

IMPLEMENT_PRIMARY_GAME_MODULE( FDefaultGameModuleImpl, $project_name, \"$project_name\" );" > $UNREAL_PROJECT_PATH/Source/$project_name/$project_name".cpp"