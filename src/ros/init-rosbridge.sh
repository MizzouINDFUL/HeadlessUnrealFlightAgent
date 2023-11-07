#!/bin/bash

source /opt/ros/noetic/setup.bash
roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True
