#!/bin/bash

#list all folder names in /Plugins
SCRIPTPATH="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )" #https://stackoverflow.com/questions/4774054/reliable-way-for-a-bash-script-to-get-the-full-path-to-itself
PLUGINDIR="$SCRIPTPATH/../../plugins_link"
PLUGINS=$(ls $PLUGINDIR)

#go through each plugin and link it to the project
for PLUGIN in $PLUGINS
do
    $SCRIPTPATH/unlink_plugin.sh $PLUGIN
done
