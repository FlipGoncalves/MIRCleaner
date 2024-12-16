# MIRCleaner

MIRCleaner is a robotic system designed to control a simulated mobile robot, based on the MiR 100 platform, for environment mapping and cleaning tasks. Equipped with a virtual vacuum cleaner bar, the robot efficiently cleans every traversed area.
The virtual bar is positioned at the front of the robot, 0.5m from the center (base_link) and is 0.64m wide.

MIRCleaner is the final project to the Mobile Robotics class.

## Features

- Simulated mapping and cleaning capabilities.
- Integration with ROS Noetic tools for navigation and environment interaction.
- Designed for modularity and flexibility in robotics applications.

## Installation

### Required Packages
Install the following ROS packages:
```bash
sudo apt install python3-catkin-tools ros-noetic-move-base ros-noetic-map-server
sudo apt install ros-noetic-mir-navigation ros-noetic-mir-gazebo ros-noetic-ira-laser-tools
sudo apt install ros-noetic-multirobot-map-merge ros-noetic-explore-lite
```

### Setting Up the Workspace
```bash
mkdir -p rm_cleaner_ws/src
cd rm_cleaner_ws
catkin config --extend /opt/ros/noetic
cd src
git clone https://github.com/iris-ua/iris_lama.git
cd iris_lama && git checkout d1a75d9 && cd ..
git clone https://github.com/iris-ua/iris_lama_ros.git
cd iris_lama_ros && git checkout 80790c6 && cd ..
git clone https://github.com/nunolau/rm_mir_cleaner.git
cd ..
catkin build
source devel/setup.bash
```

## Usage

### Launch the System
Set up each terminal with:
```bash
source devel/setup.bash
```

Run the following commands in separate terminals:
```bash
rosrun rm_mir_cleaner rm_cleaner
roslaunch rm_mir_cleaner move_base.launch
roslaunch mir_navigation amcl.launch
roslaunch rm_mir_cleaner start_map.launch
rosrun rm_mir_cleaner start_world.sh
roslaunch rm_mir_cleaner laserscan_multi_merger.launch
rviz
```

### Run the Mapper
Start Gazebo and then:
```bash
catkin build && source devel/setup.bash && roslaunch mir_mapper mir_mapper.launch
```

### Run the Cleaner
```bash
catkin build && source devel/setup.bash && roslaunch mir_cleaner mir_cleaner.launch
source devel/setup.bash && roslaunch mir_navigation amcl.launch
```
