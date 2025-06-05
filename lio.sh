#!/bin/bash

cmds=(
	# "ros2 bag play bag/sentry_20250404_234642_4.db3"
	"ros2 launch rm_vision_bringup vision_bringup.launch.py"
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"
	# "ros2 launch fast_lio mapping.launch.py"
	"ros2 launch point_lio point_lio.launch.py"
	"ros2 launch sensor_scan_generation sensor_scan_generation.launch.py"
	"sleep 3 &&  ros2 launch loam_interface loam_interface_launch.py"
	# "sleep 3 && ros2 launch icp_registration icp.launch.py" #yaml
	"ros2 launch terrain_analysis terrain_analysis_launch.py"
	"ros2 launch lidarscan lidarscan.launch.py"
	# "rqt_graph"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source ./devel/setup.sh;source install/setup.sh;$cmd;exec bash;"
	sleep 0.7
done
