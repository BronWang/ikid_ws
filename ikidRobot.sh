#!/bin/bash

# gnome-terminal --tab -- bash -c "roscore; exec bash"

gnome-terminal --tab -- bash -c "\
source /home/wp/ikid_ws/devel/setup.bash; \
roslaunch ikid_robot my_launch.launch; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 12s; \
source /home/wp/ikid_ws/devel/setup.bash; \
rosrun ikid_motion_control robot_walk_node; \
exec bash"

# 要保证 serial_port节点先于robot_walk_node启动
gnome-terminal --tab -- bash -c "\
sleep 10s; \
source /home/wp/ikid_ws/devel/setup.bash; \
rosrun ros_socket serial_port; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 10s; \
source /home/wp/ikid_ws/devel/setup.bash; \
rosrun ikid_key_ctrl key_ctrl_node; \
exec bash"
