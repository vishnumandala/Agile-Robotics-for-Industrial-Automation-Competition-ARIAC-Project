# Final Project Ariac
This repo is to collaborate with teammates to accomplish assignment goals.

## Team Members:

1. Ankur Mahesh Chavan 
2. Datta Lohith Gannavarapu 
3. Shail Kiritkumar Shah
4. Vinay Krishna Bukka
5. Vishnu Mandala
## Instructions


- Clone the package to the ariac workspace / copy the folder final_group1 into ariac_ws. Also make sure robot_commander_msgs folder is present with final_group1 in workspace src folder
- Add the `final_spring2024.yaml` file to your `ariac_gazebo` package in `config/trails/`
- Build the workspace and source using following commands
```bash
    # Install YOLO library
    sudo pip install ultralytics
    # Source ros2
    source /opt/ros/galactic/setup.bash
    # Move to ariac directory
    cd ariac_ws 
    # To install all the dependencies
    rosdep install --from-paths src -y --ignore-src
    # Build all packages i.e. ariac and rwa because '.yaml' file has 
    # been added to ariac and a new package of rwa needs to be built.
    colcon build 
    # Source the workspace
    source install/setup.bash
```
### Terminal 1
```bash
    ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=final_group1 sensor_config:=sensors trial_name:=final_spring2024
```
### Terminal 2
```bash
    ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py
```
### Terminal 3
```bash
    ros2 launch final_group1 final_group1.launch.py
```

`Note`: Run the Command in terminal two after the statement `"You can now start the competition!"` in Terminal 1.

## Results: 
You can see the ARIAC environment performing the task sequentially and the terminal 1 and terminal 3 displaying important updates
