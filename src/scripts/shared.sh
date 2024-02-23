#this shell script will contain functions related to the Unreal Launcher framework
#use source on this to get access to config variables and functions

#a varialbe that contains the path to the scripts directory
HOME_DIR=$(dirname $(realpath $0))

#Credits: https://stackoverflow.com/questions/5014632/how-can-i-parse-a-yaml-file-from-a-linux-shell-script
function parse_yaml {
   local prefix=$2
   local s='[[:space:]]*' w='[a-zA-Z0-9_]*' fs=$(echo @|tr @ '\034')
   sed -ne "/^---/d" \
        -e "s|,$s\]$s\$|]|" \
        -e "s|#[^\"'].*$||" \
        -e ":1;s|^\($s\)\($w\)$s:$s\[$s\(.*\)$s,$s\(.*\)$s\]|\1\2: [\3]\n\1  - \4|;t1" \
        -e "s|^\($s\)\($w\)$s:$s\[$s\(.*\)$s\]|\1\2:\n\1  - \3|;p" $1 | \
   sed -ne "s|,$s}$s\$|}|" \
        -e ":1;s|^\($s\)-$s{$s\(.*\)$s,$s\($w\)$s:$s\(.*\)$s}|\1- {\2}\n\1  \3: \4|;t1" \
        -e    "s|^\($s\)-$s{$s\(.*\)$s}|\1-\n\1  \2|;p" | \
   sed -ne "s|^\($s\):|\1|" \
        -e "s|^\($s\)-$s[\"']\(.*\)[\"']$s\$|\1$fs$fs\2|p" \
        -e "s|^\($s\)-$s\(.*\)$s\$|\1$fs$fs\2|p" \
        -e "s|^\($s\)\($w\)$s:$s[\"']\(.*\)[\"']$s\$|\1$fs\2$fs\3|p" \
        -e "s|^\($s\)\($w\)$s:$s\(.*\)$s\$|\1$fs\2$fs\3|p" | \
   awk -F$fs '{
      indent = length($1)/2;
      vname[indent] = $2;
      for (i in vname) {if (i > indent) {delete vname[i]; idx[i]=0}}
      if(length($2)== 0){  vname[indent]= ++idx[indent] };
      if (length($3) > 0) {
         vn=""; for (i=0; i<indent; i++) { vn=(vn)(vname[i])("_")}
         printf("%s%s%s=\"%s\"\n", "'$prefix'",vn, vname[indent], $3);
      }
   }'
}

#this function will add a new tracker to the orchestrator
function bind_script_to_event {
   #check if the SIM session exists
   if ! tmux has-session -t $SESSIONNAME 2>/dev/null; then
      echo "SIM session does not exist. Please start Unreal Launcher first!"
      exit 1
   fi

   if [ -z "$1" ]; then
      echo "Trigger phrase is empty"
      exit 1
   fi

   if [ -z "$2" ]; then
      echo "Script path is empty"
      exit 1
   fi

   CLEARLOG=false

   if [ ! -z "$3" ]; then
      if [ "$3" == "true" ]; then
         CLEARLOG=true
      fi
   fi

   #check if the orchestrator window exists
   if ! tmux list-windows -t $SESSIONNAME | grep -q "Orchestrator"; then
      echo "Orchestrator window does not exist. Please start Unreal Launcher first!"
      exit 1
   fi

   #focus on the Orchestrator window
   tmux select-window -t $SESSIONNAME:Orchestrator
   #focus on the initial pane of the orchestrator window + split it vertically and run the script in the new pane
   sleep 0.05

   #ddo split-window -h if there is only one pane, otherwise do split-window -v

   tmux send-keys -t $SESSIONNAME:Orchestrator "tmux split-window -h" Enter

   sleep 0.2
   # tmux send-keys -t $SESSIONNAME:Orchestrator "tmux rename-window -t \$TMUX_PANE $PANENAME" Enter
   tmux send-keys -t $SESSIONNAME:Orchestrator "$UELAUNCHER_HOME/src/scripts/unreal/unreal_tracker.sh \"$1\" \"$2\" \"$CLEARLOG\"" Enter
   sleep 0.1
   #switch focus back to the first pane
   tmux select-pane -t $SESSIONNAME:Orchestrator.0
   tmux resize-pane -t $SESSIONNAME:Orchestrator.0 -x 30
   tmux select-layout -t $SESSIONNAME:Orchestrator even-horizontal
}

 function get_num_lives_lived {
   echo $(ls -l $UELAUNCHER_HOME/bags/$SIM_START_DATE/ | grep -c ^d)
 }