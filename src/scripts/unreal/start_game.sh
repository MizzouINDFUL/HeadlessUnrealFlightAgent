
sleep 7;
tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal 'py run_mrq.py'" C-m