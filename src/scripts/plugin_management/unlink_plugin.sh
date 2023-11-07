#!/bin/bash

SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

eval $($SCRIPTPATH/../eval_yml.sh $SCRIPTPATH/../../config.yml)

PROJECTDIR=$unreal_project
PROJECTNAME=$(basename "$PROJECTDIR")
#projectdir without projectname
PROJECTDIR=${PROJECTDIR%$PROJECTNAME}
#remove last symbol in projectdir
PROJECTDIR=${PROJECTDIR%?}

#remove the symbolic link to the plugin in the projects Plugins/ directory
if [ -L "$PROJECTDIR/Plugins/$1" ]; then
    rm "$PROJECTDIR/Plugins/$1"
fi
