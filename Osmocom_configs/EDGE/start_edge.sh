#!/bin/bash
SESSION="gsm_edge"

tmux new-session -d -s $SESSION -n 'EDGE'

tmux set-option -s -t $SESSION mouse on

#2
tmux split-window -v -t $SESSION:0

#2
tmux split-window -v -t $SESSION:0


#Align
tmux select-layout -t $SESSION:0 even-vertical

tmux send-keys -t $SESSION:0.0 'osmo-sgsn -c osmo-sgsn.cfg' Enter

tmux send-keys -t $SESSION:0.1 'osmo-ggsn -c osmo-ggsn.cfg' Enter

tmux send-keys -t $SESSION:0.2 'osmo-pcu -c osmo-pcu.cfg' Enter

tmux attach-session -t $SESSION
