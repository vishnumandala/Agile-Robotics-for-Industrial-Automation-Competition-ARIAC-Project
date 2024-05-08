#!/bin/zsh

# Use the provided argument as the workspace path, or default to the current directory
ws_path=${1:-$(pwd)}

# Open three Terminator windows and run commands in them, sourcing environments in each
echo "Launching Terminator windows..."

terminator -T "Ariac Gazebo" -e "zsh -c 'source /opt/ros/galactic/setup.zsh; source $ws_path/install/setup.zsh; clear; ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=final_group1 sensor_config:=sensors trial_name:=test'" &

terminator -T "Robot Controller" -e "zsh -c 'sleep 1; source /opt/ros/galactic/setup.zsh; source $ws_path/install/setup.zsh; clear; ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py'" &

terminator -T "Competition Node" -e "zsh -c 'sleep 10; source /opt/ros/galactic/setup.zsh; cd $ws_path; source install/setup.zsh; clear; ros2 launch final_group1 final_group1.launch.py'" &

