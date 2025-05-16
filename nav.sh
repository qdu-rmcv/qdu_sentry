#!/bin/bash

cmds=(
	"ros2 launch rm_vision_bringup vision_bringup.launch.py "
	# "ros2 launch rm_description robot_description.launch.py"
	"ros2 launch livox_ros_driver2 msg_MID360_launch.py"

	# "ros2 launch fast_lio mapping.launch.py"

	"ros2 launch point_lio point_lio.launch.py"
	"ros2 launch sensor_scan_generation sensor_scan_generation.launch.py"
	"ros2 launch loam_interface loam_interface_launch.py"

	"ros2 launch terrain_analysis terrain_analysis_launch.py" #cpp
	"ros2 launch icp_registration icp.launch.py" #yaml
	# "ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py"

	"ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
	# "ros2 launch lidarscan lidarscan.launch.py"

	"ros2 launch gimbalsend gimbalsend.launch.py"
	# "ros2 run rm_decision rm_decision_node"
	"sleep 3 && ros2 launch rm_navigation bringup_launch.py"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source install/setup.bash;$cmd;exec bash;"
	sleep 0.7
done
