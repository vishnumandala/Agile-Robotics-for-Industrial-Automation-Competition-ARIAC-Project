# rwa3_ariac
This repo is to collaborate with teammates to accomplish assignment goals.

Task 3 - C++
Task 4 - Python

## Instructions

1. Clone the package to the ariac workspace.
2. Assuming ariac main package is alread built, use below command to build our package

```
colcon build --packages-select rwa3_group1
```

3. Source the workspace
4. Terminal 1:
```bash
ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa3_spring2024

```
5. Terminal 2:
```bash
ros2 run rwa3_group1 start_ariac_competition 
```