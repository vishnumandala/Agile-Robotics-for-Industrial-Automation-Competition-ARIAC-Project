# rwa3_ariac
This repo is to collaborate with teammates to accomplish assignment goals.

Task 3 - C++

Task 4 - Python

Task 5 - Python

Task 6 - Python

Task 7 - Python

Task 8 - Python
## Team Members:

1. Ankur Mahesh Chavan 
2. Datta Lohith Gannavarapu 
3. Shail Kiritkumar Shah
4. Vinay Krishna Bukka
5. Vishnu Mandala
## Instructions

- Clone the package to the ariac workspace / copy the folder rwa3_group1 into ariac_ws
- Add the `rwa3_spring2024.yaml` file to your `ariac_gazebo` package in `config/trails/`
- Build the workspace and source using following commands
```bash
    source /opt/ros/galactic/setup.bash
    # Move to ariac directory
    cd ariac_ws 
    # build the assignment package
    colcon build --packages-select rwa3_group1
    # Source the workspace
    source install/setup.bash
```
### Terminal 1
```bash
    ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3_spring2024
```
### Terminal 2
```bash
    ros2 launch rwa3_group1 rwa3_group1.launch.py
```

`Note`: Run the Command in terminal two after the statement `"You can now start the competetion!"` in Terminal 1.

## Results: 
You can see the ARIAC environment performing the task sequentially and the terminal 2 displaying important updates