#temp
# tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py add_simple_sky.py 0 1'" C-m
sleep 7;
# tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py unreal.MindfulLib.start_life()'" C-m
tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py run_mrq.py'" C-m