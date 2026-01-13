# Useful ros2_kortex Commands
1. Launch a simulated Gen3:
```
ros2 launch kortex_bringup gen3.launch.py robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true gripper:="robotiq_2f_85"
```

2. Connect with a Gen3:
```
ros2 launch kortex_bringup gen3.launch.py robot_ip:=192.168.1.127 gripper:="robotiq_2f_85" launch_rviz:=false
```
Note that **admittance mode remains disabled** until ros2_kortex is terminated.

3. Launch realsense-ros:
```
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
```

4. Activate / deactivate twist_controller:
```
# Activate the twist_controller, joint_trajectory_controller needs to be disabled for a physical robot:
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
    activate_controllers: [twist_controller],
    deactivate_controllers: [joint_trajectory_controller],
    strictness: 1,
    activate_asap: true,
}"

# deactivate twist_controller:
ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{
    activate_controllers: [joint_trajectory_controller],
    deactivate_controllers: [twist_controller],
    strictness: 1,
    activate_asap: true,
}"
```

5. Publish a waypoint in Joint Space
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/JointTrajectory "{
    joint_names: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7],
    points: [
        { positions: [0, 0, 0, 0, 0, 0, 0], time_from_start: { sec: 10 } },
    ]
}" -1
```

6. Send a gripper command
```
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command:{position: 0.0, max_effort: 100.0}}"
```