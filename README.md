# MIR Cleaner
In this assignment you should develop 2 robotic agents to command a simulated mobile robot that should: map or clean the environment using an existing map.

The simulated robot is based on the Mobile Intelligent Robots MiR 100 platform enhanced with a virtual vacuum cleaner bar placed at front of the robot that cleans every space it traverses. The virtual bar is positioned at the front of the robot, 0.5m from the center (base_link) and is 0.64m wide.

## Packages to Install
```
sudo apt install python3-catkin-tools
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-mir-navigation
sudo apt install ros-noetic-mir-gazebo
sudo apt install ros-noetic-ira-laser-tools
sudo apt install ros-noetic-multirobot-map-merge ros-noetic-explore-lite
```

## Creating the Repo
```
mkdir -p rm_cleaner_ws/src
cd rm_cleaner_ws
catkin config --extend /opt/ros/noetic
cd src
git clone git@github.com:iris-ua/iris_lama.git
cd iris_lama
git checkout d1a75d9
cd ..
git clone git@github.com:iris-ua/iris_lama_ros.git
cd iris_lama_ros
git checkout 80790c6
cd ..
git clone git@github.com:nunolau/rm_mir_cleaner.git
cd ..
catkin build
. devel/setup.bash
```

## Running the Code
Do this for every terminal
```
. devel/setup.bash
```

In different terminals do
```
rosrun rm_mir_cleaner rm_cleaner
roslaunch rm_mir_cleaner move_base.launch
roslaunch mir_navigation amcl.launch
roslaunch rm_mir_cleaner start_map.launch
rosrun rm_mir_cleaner start_world.sh
roslaunch rm_mir_cleaner laserscan_multi_merger.launch
rviz
```

### Running the Mapper 
```
roslaunch mir_gazebo mir_maze_world.launch
roslaunch rm_mir_cleaner move_base.launch
rosrun mir_mapper mir_mapper_node.py
rviz
```

COLCOAR O GAZEBO A DAR NO PLAY

```
catkin build && . devel/setup.bash && clear && roslaunch mir_mapper mir_mapper.launch
```

### Saving the Map
```
rosrun map_server map_saver -f mapname
```