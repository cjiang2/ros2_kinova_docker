# ros2_kortex_contrib
ROS 2 Docker and a custom ros-driver for interfacing with Kinova Gen3

## Building & Running
Modify the user settings inside the [Dockerfile](https://github.com/cjiang2/ros2_kortex_contrib/blob/humble/Dockerfile#L31) as needed. Modify the [docker-compose](https://github.com/cjiang2/ros2_kortex_contrib/blob/humble/docker-compose.yml) to control mounting, user access, etc. 

Then, build with docker-compose:
```
docker compose build --no-cache
```

Spin the docker and attach a shell:

```
docker compose up -d
docker exec -it humble-kortex /bin/bash
```

Make sure *.rules are in "/etc/udev/rules.d" on host.

## Initialize ROS2 Workspace
Clone [ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex/tree/humble) and [realsense-ros](https://github.com/realsenseai/realsense-ros/tree/4.56.4) to your destinated **$COLCON_WS**.

Note that for [realsense-ros](https://github.com/realsenseai/realsense-ros/tree/4.56.4), versions need to match the upstream librealsense2:
- librealsense2 2.57.0 -> use up-to-date branch
- librealsense2 2.56.4 -> use 4.56.4

For ros2_kortex, pull relevant packages:
```
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/ros2_kortex-not-released.$ROS_DISTRO.repos
vcs import src --skip-existing --input src/ros2_kortex/simulation.humble.repos
```

Initialize your $COLCON_WS with (prevent rosdep to fetch librealsense2 twice):
```
rosdep update
sudo apt-get update && rosdep install --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y -r
```

Build your **$COLCON_WS**:
```
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Commit your docker after installing the packages:
```
docker commit humble-kortex ros/humble-kortex
```

## kortex_contrib
kortex_contrib is a wrapper around kortex-api to issue ROS1-like high-level momvement commands (velocity control, synchronized angular and cartesian actions, etc). 

Only one of kortex_contrib or ros2_kortex should be launched to avoid racing for controlling the robot. 

To use the kortex_contrib, copy both **kortex_contrib** and **kortex_contrib_msgs** under the **$COLCON_WS/src**, then build: 

```
colcon build --symlink-install --packages-select kortex_contrib kortex_contrib_msgs
```

Communicate with a real-world Gen3 robot:
```
ros2 launch kortex_contrib gen3.launch.py
```

(Expanded from [this](https://github.com/RRL-ALeRT/kinova_stuffs/tree/master) amazing yet simple controller)

Check the [List-of-Commands](https://github.com/cjiang2/ros2_kortex_contrib/blob/humble/cmd.md) for interfacing with the robot.