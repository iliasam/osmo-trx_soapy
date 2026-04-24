#!/bin/bash
SESSION="gsm_master"

tmux new-session -d -s $SESSION -n 'VOICE' -x 200 -y 100

tmux set-option -s -t $SESSION mouse on

#2
tmux split-window -v -t $SESSION:0
#3
tmux split-window -v -t $SESSION:0
#4
tmux split-window -v -t $SESSION:0
#5
tmux split-window -v -t $SESSION:0



#Align
tmux select-layout -t $SESSION:0 even-vertical

tmux select-layout -t $SESSION:0 tiled

tmux send-keys -t $SESSION:0.0 'osmo-hlr  -c osmo-hlr.cfg' Enter
tmux send-keys -t $SESSION:0.1 'osmo-stp -c osmo-stp.cfg' Enter
tmux send-keys -t $SESSION:0.2 'osmo-msc -c osmo-msc.cfg' Enter
sleep 1
tmux send-keys -t $SESSION:0.3 'osmo-mgw -c osmo-mgw.cfg' Enter
sleep 2
tmux send-keys -t $SESSION:0.4 'osmo-bsc -c osmo-bsc.cfg' Enter

tmux attach-session -t $SESSION
