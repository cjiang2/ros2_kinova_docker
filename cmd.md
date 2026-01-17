# Commands for ros2_kortex 
### Launch a simulated Gen3
```
ros2 launch kortex_bringup gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:="robotiq_2f_85"
```

### Connect with a Gen3
```
ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.127 gripper:="robotiq_2f_85" launch_rviz:=false
```
Note that **admittance mode remains disabled** until ros2_kortex is terminated (See [#209](https://github.com/Kinovarobotics/ros2_kortex/issues/209)). 

### Launch realsense-ros:
```
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

### Activate / deactivate twist_controller:
```
# Activate twist_controller
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
    activate_controllers: [twist_controller],
    deactivate_controllers: [joint_trajectory_controller],
    strictness: 1,
    activate_asap: true,
}"

# Activate joint_trajectory_controller
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
    activate_controllers: [joint_trajectory_controller],
    deactivate_controllers: [twist_controller],
    strictness: 1,
    activate_asap: true,
}"
```

### Publish a waypoint in Joint Space
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [
        { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
    ]
}" -1
```

### Send a gripper command
```
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"
```


# Commands for kortex_contrib
kortex_contrib provides additional services and topics for interfacing with Gen3.

Similar action server like "/robotiq_gripper_controller/gripper_cmd" and publishers are provided to match ros2_kortex as closely as possible.

### Connect with a Gen3
```
ros2 launch kortex_contrib gen3.launch.py
```
NOTE: Launch one of ros2_kortex or kortex_contrib's interface.

### Send a joint velocity command
```
ros2 topic pub /joint_vel_cmd control_msgs/JointJog "{
    velocities: [0.1, 0, 0, 0, 0, 0, 0],
}" -1
```

### Stop the robot
```
ros2 topic pub /joint_vel_cmd control_msgs/JointJog "{
    velocities: [0.0, 0, 0, 0, 0, 0, 0],
}" -1

# or use the /stop topic
ros2 topic pub --once /stop std_msgs/msg/Empty "{}"
```

### Send a joint angle, blocking until finished
```
ros2 service call /send_joint_trajectory kortex_contrib_msgs/srv/SendJointTrajectory "{
  traj: {
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [
        { positions: [1.339, 5.9636, 3.1054, 4.017, 6.282, 0.0198, 1.5694], time_from_start: { sec: 0 } },
    ]
  }
}"
```

### Send a cartesian pose, blocking until finished
```
# Pose: (x, y, y, theta_x, theta_y, theta_z)
ros2 service call /send_cartesian_pose kortex_contrib_msgs/srv/SendCartesianPose "{
  pose: {
    x: 0.52, 
    y: -0.2,
    z: 0.5202783942222595,
    theta_x: 87.01734924316406,
    theta_y: -0.4342148005962372,
    theta_z: 89.64740753173828,
  }
}"
```