#!/bin/bash

SESSION="teleop_session"

tmux kill-session -t $SESSION 2>/dev/null

tmux new-session -d -s $SESSION -n "Planning"

INIT_CMD="export ROS_DOMAIN_ID=211; source ~/Imitation_learning-Proj/teleop_ws/install/setup.bash"


tmux send-keys -t $SESSION:0 "$INIT_CMD && ros2 launch start_up start.launch.py" C-m


tmux new-window -t $SESSION -n "Controlling"
tmux split-window -h -t $SESSION:1.0
tmux split-window -v -t $SESSION:1.1


tmux send-keys -t $SESSION:1.0 "$INIT_CMD && ros2 run wiseglove18 glove_data_pub" C-m
tmux send-keys -t $SESSION:1.1 "$INIT_CMD && ros2 run inspire_hand inspire_teleop" C-m
tmux send-keys -t $SESSION:1.2 "$INIT_CMD && ros2 run rokae_rt rokae_rt_node" C-m


tmux new-window -t $SESSION -n "Data collection"
tmux send-keys -t $SESSION:2 "$INIT_CMD && ros2 run data_collection data_collect"


tmux select-layout -t $SESSION:1 tiled


tmux select-window -t $SESSION:1.0
tmux attach-session -t $SESSION
