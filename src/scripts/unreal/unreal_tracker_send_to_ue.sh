#this this look at Unreal.log in a while loop looking for a trigger phrase (argument 1). Once the phrase is detected, it will run the COMMAND (argument 2) and then exit. Argument 3 should be a boolean that indicates whether the COMMAND should clear the log file after running the COMMAND

TRIGGER=$1
COMMAND=$2
CLEARLOG=false
if [ "$3" = true ]; then
    CLEARLOG=true
fi

#check if trigger is empty
if [ -z "$TRIGGER" ]; then
    echo "Trigger phrase is empty"
    exit 1
fi

echo "session: $SESSIONNAME"
echo "trigger phrase: '$TRIGGER'"
echo "binded command: $COMMAND"

while true; do
    if [ -f $UELAUNCHER_HOME/src/logs/$SESSIONNAME-Unreal.log ]; then
        if grep -q "$TRIGGER" $UELAUNCHER_HOME/src/logs/$SESSIONNAME-Unreal.log; then
        
            echo "Trigger phrase detected. Running command $COMMAND"
            TELLUNREAL_PORT=$(yq e '.ports_to_reserve[0].tellunreal_listener' tmp/$SESSIONNAME-config.yml)
            python3 src/scripts/unreal/send_to_unreal.py $TELLUNREAL_PORT "$COMMAND"
            
            if [ "$CLEARLOG" = true ]; then
                echo "Clearing log file"
                echo "" > $UELAUNCHER_HOME/src/logs/$SESSIONNAME-Unreal.log
            fi

            tmux kill-pane -t $TMUX_PANE
            exit 0
        fi
    fi
    sleep 1
done