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

To try sending a command using `ur_robot_driver` package,
```
ros2 run ur_robot_driver example_move.py
```

If `my_mtc` package is installed (in my repos), run the following alongside launch with Moveit,
```
ros2 launch my_mtc pick_place_demo.launch.py # currently not working, see known bugs section
```

### Additional Details
Note that certain changes need to be made to other packages that this package depends on. (There's too many lines to keep track off, so I will only provide a summary)
1. Universal_Robots_ROS2_Driver (`ur_moveit_config` package)
- `controllers.yaml`: Added gripper_controller config. (produced by moveit_assistant)
- `joint_limits.yaml`: Added gripper limits. (produced by ...)
- `ur_macro.srdf.xacro`: Added gripper groups, states and disabled collisions (produced by moveit_assistant)
- `pilz_cartesian_limits.yaml`: Added new file (produced by moveit_assistant)
- `ur_moveit.launch.py`: Added move_group_capabilities
2. `gz_ros2_control` package
- `gz_system.cpp`: Removed adding suffix `_mimic` when `ros2_control` sees a mimic joint. [WHO IN THE RIGHT MIND IMPLEMENTED THIS, EXPLAIN YOURSELF]
3. Universal_Robots_ROS2_Driver `ur_robot_driver` package
- `example_move.py` script: Instead of `scaled_joint_trajactory_controller`, changed to `joint_trajectory_controller`


### Known bugs
- [ ] Planned path/movement of robot arm seems "janky". Gripper approaching in wrong direction [partially fixed]
- [x] Unable to move to "set positions by srdf" using moveit task constructor [fixed by point 1.5, move_group_capabilities]
- [ ] Unable to move using moveit task constructor after planning [Linked with below bug? Looks like there is a path planned out? But no clue...] 
- [ ] When used alongside `my_mtc` package, optimizer is unable to find a "good enough" path. [Collision and smootheness cost is too high]
- [x] Gripper closing and opening is werid. Mimic joints don't seem to work as intended in Gazebo. [nuked gripper code, redo using FollowJointTrajectory instead of GripperCommand]
- [x] `ros2_control` can't find joint with added `_mimic` [Install source, change the code]
- [x] Cannot control gripper with moveit [Use moveit assistant to produce controllers, groups etc.]
- [x] Cannot control arm with moveit [Disable collisions between arm and gripper]
- [x] Moveit planning sometimes goes through ground plane [Add ground plane box to urdf]

