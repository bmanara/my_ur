# UR Robot with Robotiq Gripper
Currently in progress, unstable. Only has UR10 robot for now (without gripper)

Note: Required to have Universal_Robots_ROS2_Driver package installed. (I think...)

After adding to workspace, run
```
colcon build --packages-select my_ur
source install/setup.bash
```

To run the ROS2 UR Description Launch File, (ur10 only)
```
ros2 launch my_ur view_ur.launch.py ur_type:=ur10
```

To run with ROS UR Simulation Launch File, (ur10 only)
```
ros2 launch my_ur ur_sim_control.launch.py # Launch only rviz and gazebo
ros2 launch my_ur ur_sim_moveit.launch.py # Launch with Moveit
```
