# Enabling Multi-Robot ARM in Gazebo for ROS2
This git repository presents an ROS2 package that demonstrates the simultaneous spawning of multiple robotic arms in a Gazebo simulation. Historically, such an application is difficult to come by within the ROS2 community, leaving a knowledge gap for developers looking to implement multi-arm robotic systems. The work outlined here not only fills this void but also offers a practical guide for beginners and interested parties looking to replicate similar multi-robot simulations in Gazebo.

The UR5 robotic arm, a versatile and widely-used model in robotics, serves as the core of this demonstration. By focusing on this particular model, the tutorial ensures a broad relevance to a multitude of potential robotic applications.

![image](https://github.com/arshadlab/multi_robot_arm/assets/85929438/4e052e79-65c7-4fe5-b73a-871b76b9f01e)
![image](https://github.com/arshadlab/multi_robot_arm/assets/85929438/3189c420-33c1-424c-aaa4-0cc2b8cc868c)

The present version of this tutorial employs a fork of pymoveit2, a Python library developed to facilitate interaction with MoveIt2. This has been an effective approach and served the purpose well.

However, in recent times, MoveIt2 has introduced native Python bindings in its codebase. These bindings enable direct interaction with MoveIt2 via Python, eliminating the need for additional libraries like pymoveit2. This advancement simplifies the setup process, reduces dependency issues, and potentially enhances performance and stability.

As part of ongoing improvements and in the spirit of keeping up with these updates, future versions of this tutorial may transition to using the Python bindings provided directly by MoveIt2. This would replace the current reliance on the pymoveit2 fork, streamlining the implementation process and ensuring compatibility with future developments in MoveIt2. Consequently, this change will not only refine the tutorial but also make it more adaptable and robust for future use-cases.

## Setup
The package is verified with ROS2 Foxy and Humble.

Dependencies: ROS2 [Foxy or Humble], moveit2, pymoveit2, Gazebo 11

## Clone and Build
```
source /opt/ros/<DISTRO>/setup.bash
mkdir -p robot_ws/src
cd robot_ws/src
git clone https://github.com/arshadlab/multi_robot_arm.git
git clone https://github.com/arshadlab/pymoveit2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## Launch
### Console A (Launch Gazebo)
```
source ./install/setup.bash
ros2 launch multi_robot_arm gazebo_arm.launch.py
```

### Console B (Trigger ARM movement)

ARM1 movement to position [0.5, 0.4,0.2] using kinematic path planner
```
source ./install/setup.bash
cd ./src/pymoveit2/examples
python ex_pose_goal.py --ros-args -r __ns:=/arm1 -p position:=[0.5, 0.4,0.2]
```

ARM4 movement to position [0.5, 0.4,0.2] using cartesian path planner
```
source ./install/setup.bash
cd ./src/pymoveit2/examples
python ex_pose_goal.py --ros-args -r __ns:=/arm4 -p position:=[0.5, 0.4,0.2] -p cartesian:=True
```

__ns:=/namespace parameter is to direct commands to a specific instance of robot. Position coordinates are given relative to arm position.

## Acknowledgement
pymoveit2 -> https://github.com/AndrejOrsula/pymoveit2
