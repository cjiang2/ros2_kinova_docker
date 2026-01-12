# ros2_kinova_docker
ROS 2 Docker for Interfacing with Kinova Gen3

## Building & Running
Modify the user settings inside the [Dockerfile](https://github.com/cjiang2/ros2_kinova_docker/blob/humble/Dockerfile#L31) as needed. Then, build with docker-compose:
```
docker compose build --no-cache
```

Modify the [docker-compose](https://github.com/cjiang2/ros2_kinova_docker/blob/humble/docker-compose.yml) to control mounting, user access, etc.

Then, spin the docker and attach a shell:

```
docker compose up -d
docker exec -it humble-kortex /bin/bash
```

## Initialize ROS2 Workspace
Clone [ros2_kortex](https://github.com/Kinovarobotics/ros2_kortex/tree/humble) and [realsense-ros](https://github.com/realsenseai/realsense-ros/tree/4.56.4) to your destinated **$COLCON_WS**.

Note that for [realsense-ros](https://github.com/realsenseai/realsense-ros/tree/4.56.4), versions need to match the upstream librealsense2:
- librealsense2 2.57.0 -> use up-to-date branch
- librealsense2 2.56.4 -> use 4.56.4

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