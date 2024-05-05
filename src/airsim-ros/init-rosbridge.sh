#!/bin/bash

CUSTOM_PORT=""
NODENAME=""

if [ -n "$2" ]; then
    NODENAME=$2
    echo "rosbridge is going to use a custom node name: $NODENAME"
else
    echo "rosbridge is going to use the default node name"
fi

sed -i "s/name=\"rosbridge_tcp\"/name=\"rosbridge_tcp_$NODENAME\"/g" /opt/ros/noetic/share/rosbridge_server/launch/rosbridge_tcp.launch
sed -i "s/name=\"rosapi\"/name=\"rosapi_$NODENAME\"/g" /opt/ros/noetic/share/rosbridge_server/launch/rosbridge_tcp.launch


source /opt/ros/noetic/setup.bash
if [ -n "$1" ]; then
    CUSTOM_PORT=$1
    echo "rosbridge is going to use a custom port: $CUSTOM_PORT"
    roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True port:=$CUSTOM_PORT
else
    echo "rosbridge is going to use the default port"
    roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
fi

