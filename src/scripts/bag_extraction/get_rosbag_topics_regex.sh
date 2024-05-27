#this will go through all file names in $UELAUNCHER_HOME/src/scripts/bag_extraction/topics, remove .py at the and create a regular expression for the command rosbag record to record all topics that match the file names in that folder
#for example, if the file names are: _unreal_ros_ground_truth.py, _unreal_ros_image_color.py, _unreal_ros_image_depth.py, this should output ((.*)unreal(.*)ground(.*)truth(.*)|(.*)unreal(.*)image(.*)color(.*)|(.*)unreal(.*)image(.*)depth(.*))

source $UELAUNCHER_HOME/src/scripts/shared.sh
# eval $(parse_yaml $UELAUNCHER_HOME/config.yml)

#get all file names
topics_regex=""
for file in $UELAUNCHER_HOME/src/scripts/bag_extraction/topics/*.py; do
    #get the name of the file without the full path
    file=${file##*/}
    #remove .py from file name
    file=${file%.py}
    #replace _ with (.*) and add | at the end
    file=${file//_/(.*)}
    #add file name to topics_regex
    topics_regex="$topics_regex$file|"
done

#remove last | from topics_regex
topics_regex=${topics_regex%?}

#put it all in parentheses
topics_regex="($topics_regex)"

echo $topics_regex