#!/bin/bash
{
	gnome-terminal --tab "XXD_ros" -- bash -c "ros2 launch turtlebot4_navigation localization.launch.py map:=/home/max-unterwegs/ros2test/src/map.yaml;exec bash"
 
}&
 
sleep 2s
{
	gnome-terminal --tab "XXD_demo" -- bash -c "ros2 launch turtlebot4_navigation nav2.launch.py;exec bash"
}&
 
sleep 4s
{
	 gnome-terminal --tab "XXD_fk" -- bash -c "ros2 launch turtlebot4_viz view_robot.launch.py;exec bash"
        # gnome-terminal --tab "XXD_ros" -- bash -c 'export RUN_AFTER_BASHRC="rosrun hr_task_planning  plan_arm_fk_node;exec bash"'
}
