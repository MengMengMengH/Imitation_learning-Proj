#!/bin/bash

SESSION="teleop_session"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n "Control_Center"

INIT_CMD="export ROS_DOMAIN_ID=211; source ~/Imitation_learning-Proj/teleop_ws/install/setup.bash"

tmux send-keys -t $SESSION:0.0 "$INIT_CMD && ros2 launch start_up start.launch.py" C-m
tmux split-window -h -t $SESSION:0.0
tmux split-window -v -t $SESSION:0.0

tmux split-window -h -t $SESSION:0.2
tmux split-window -h -t $SESSION:0.3

tmux send-keys -t $SESSION:0.2 "$INIT_CMD && ros2 run wiseglove18 glove_data_pub" C-m

tmux send-keys -t $SESSION:0.3 "$INIT_CMD && ros2 run inspire_hand inspire_teleop" C-m

tmux send-keys -t $SESSION:0.4 "$INIT_CMD && ros2 run rokae_rt rokae_rt_node" C-m

tmux send-keys -t $SESSION:0.1 "$INIT_CMD && ros2 run data_collection data_collect"


tmux select-layout -t $SESSION:0 tiled

tmux select-pane -t $SESSION:0.0
tmux attach-session -t $SESSION