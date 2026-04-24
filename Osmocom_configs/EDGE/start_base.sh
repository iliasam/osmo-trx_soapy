#!/bin/bash
SESSION="gsm_base"

tmux new-session -d -s $SESSION -n 'BASE'

tmux set-option -s -t $SESSION mouse on

#2
tmux split-window -v -t $SESSION:0

#Align
tmux select-layout -t $SESSION:0 even-vertical

tmux send-keys -t $SESSION:0.0 'osmo-bts-trx -c osmo-bts-trx.cfg' Enter

tmux send-keys -t $SESSION:0.1 'DIR=/home/user1/GSM/osmo_trx_soapy/Transceiver52M' Enter
tmux send-keys -t $SESSION:0.1 '$DIR/osmo-trx-soapy -C $DIR/test1.cfg' Enter

tmux attach-session -t $SESSION
