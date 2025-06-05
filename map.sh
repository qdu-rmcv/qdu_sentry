#!/bin/bash

cmds=(

	"ros2 launch pcd2pgm pcd2pgm.launch.py"
	# "rviz2"
	"sleep 4 && ros2 run nav2_map_server map_saver_cli -f map"
	)

for cmd in "${cmds[@]}";
do
	echo Current CMD : "$cmd"
	gnome-terminal -- bash -c "cd $(pwd);source ./devel/setup.bash;source install/setup.bash;$cmd;exec bash;"
	sleep 0.7
done
