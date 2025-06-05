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

### Additional Details
Note that certain changes need to be made to other packages that this package depends on. (There's too many lines to keep track off, so I will only provide a summary)
1. Universal_Robots_ROS2_Driver (`ur_moveit_config` package)
- `controllers.yaml`: Added gripper_controller config. (produced by moveit_assistant)
- `joint_limits.yaml`: Added gripper limits. (produced by ...)
- `ur_macro.srdf.xacro`: Added gripper groups, states and disabled collisions (produced by moveit_assistant)
2. `gz_ros2_control` package
- `gz_system.cpp`: Removed adding suffix `_mimic` when `ros2_control` sees a mimic joint. [WHO IN THE RIGHT MIND IMPLEMENTED THIS, EXPLAIN YOURSELF]


### Known bugs
- Gripper closing and opening is werid. Mimic joints don't seem to work as intended in Gazebo. (Absolutely no idea why it's doing this ._.)
- `ros2_control` can't find joint with added `_mimic` (fixed)
- Cannot control gripper with moveit (fixed)
- Cannot control arm with moveit (fixed)

