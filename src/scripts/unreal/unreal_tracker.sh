#this this look at Unreal.log in a while loop looking for a trigger phrase (argument 1). Once the phrase is detected, it will run the script (argument 2) and then exit. Argument 3 should be a boolean that indicates whether the script should clear the log file after running the script

TRIGGER=$1
SCRIPT=$2
CLEARLOG=false
if [ "$3" = true ]; then
    CLEARLOG=true
fi

#check if trigger is empty
if [ -z "$TRIGGER" ]; then
    echo "Trigger phrase is empty"
    exit 1
fi

echo "trigger phrase: '$TRIGGER'"
echo "binded script: $SCRIPT"

while true; do
    if [ -f $UELAUNCHER_HOME/src/logs/Unreal.log ]; then
        if grep -q "$TRIGGER" $UELAUNCHER_HOME/src/logs/Unreal.log; then
            echo "Trigger phrase detected. Running script $SCRIPT"
            bash $SCRIPT
            
            if [ "$CLEARLOG" = true ]; then
                echo "Clearing log file"
                echo "" > $UELAUNCHER_HOME/src/logs/Unreal.log
            fi

            tmux kill-pane -t $TMUX_PANE
            exit 0
        fi
    fi
    sleep 1
done