# UR Robot with Robotiq Gripper
Currently in progress, unstable. Only has UR10 robot for now (without gripper)

After adding to workspace, run
```
colcon build --packages-select my_ur
source install/setup.bash
```

To run the ROS2 UR Description Launch File, (ur10 only)
```
ros2 launch my_ur view_ur.launch.py ur_type:=ur10
```
