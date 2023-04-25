#!/bin/bash
session="tinyquad"

tmux has-session -t $session
if [ $? != 0 ]; then
    # create new tmux session 
    tmux new-session -d -s $session

    tmux set -g mouse on

    tmux set -g status-bg black
    tmux set -g status-fg white

    # roscore
    window=0
    tmux rename-window -t $session:$window 'workspace'
    tmux send-keys -t $session:$window 'cd ~/tinyquad_ws' C-m
    tmux send-keys -t $session:$window 'roscore' C-m

    tmux split-window -v
    tmux send-keys -t $session:$window 'sleep 2 ; roslaunch tinyquad_description gazebo_with_control.launch ' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 3 ; roslaunch tinyquad_planner visualize.launch' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 5 ; roslaunch tinyquad_control tinyquad_control.launch' C-m
    tmux select-layout tiled

    tmux split-window -h
    tmux send-keys -t $session:$window 'sleep 6 ; roslaunch tinyquad_planner start.launch' C-m
    tmux select-layout tiled

fi

tmux attach-session -t $session

