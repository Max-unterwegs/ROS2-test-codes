#!/bin/bash
 
 # --- for every Terminal-tab
{
	gnome-terminal --tab "XXD_ros" -- bash -c "ros2 launch slam_gmapping slam_gmapping.launch.py;exec bash"
 
}&
 
sleep 2s
{
	gnome-terminal --tab "XXD_demo" -- bash -c "ros2 launch turtlebot4_viz view_robot.launch.py;exec bash"
}&
 
sleep 4s
{
	 gnome-terminal --tab "XXD_fk" -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard;exec bash"
        # gnome-terminal --tab "XXD_ros" -- bash -c 'export RUN_AFTER_BASHRC="rosrun hr_task_planning  plan_arm_fk_node;exec bash"'
}
