#!/bin/bash

# gnome-terminal --tab -- bash -c "roscore; exec bash"

gnome-terminal --tab -- bash -c "\
source /home/wp/ikid_ws/devel/setup.bash; \
roslaunch ikid_robot my_launch.launch; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 15s; \
source /home/wp/ikid_ws/devel/setup.bash; \
rosrun ikid_motion_control robot_walk_node; \
exec bash"

gnome-terminal --tab -- bash -c "\
sleep 15s; \
source /home/wp/ikid_ws/devel/setup.bash; \
rosrun ikid_key_ctrl key_ctrl_node; \
exec bash"
