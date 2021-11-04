#!/bin/sh
tmux new-session -s PyConLauncher -d

tmux split-window -v -t PyConLauncher:0.0
tmux send-keys "roslaunch eigenbot_driver titan_driver.launch" Enter
tmux split-window -h -t PyConLauncher:0.1

tmux send-keys "rostopic pub -1 eigenbot/in std_msgs/String \"data: 'FFZ'\"" Enter
tmux send-keys "sleep 0.5" Enter
tmux send-keys "rostopic pub -1 eigenbot/in std_msgs/String \"data: 'FFZ'\"" Enter
tmux send-keys "sleep 0.5" Enter
tmux send-keys "rostopic pub -1 eigenbot/in std_msgs/String \"data: 'FFZ'\"" Enter
tmux send-keys "rostopic echo /titan/joint/fb/position" Enter

tmux split-window -h -t PyConLauncher:0.0

tmux send-keys -t PyConLauncher:0.0 "rosrun joy joy_node" Enter
tmux send-keys -t PyConLauncher:0.1 "sleep 8" Enter
tmux send-keys -t PyConLauncher:0.1 "cd src/py_con" Enter
tmux send-keys -t PyConLauncher:0.1 "python3 titan.py ROS" Enter
tmux attach

