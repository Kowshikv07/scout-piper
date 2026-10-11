# Scout Mini + Piper Robotics Project

ROS 2 robotics project for Scout Mini and Piper simulation.

## Project contents

- `src/my_robot/` — main ROS 2 package
- `model.sdf` — workspace-level SDF asset
- `nav_world.zip` — navigation world archive
- `yolov8n.pt` — YOLO model weights

## Dependencies

- ROS 2 Jazzy
- Gazebo Sim
- Piper ROS 2 packages: https://github.com/MarqRazz/piper_ros2

## Build

Review package dependencies, then run from a ROS 2 workspace:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

See the package launch files and configuration for the appropriate run commands.
