packages to install
sudo apt install python3-catkin-tools
sudo apt install ros-noetic-move-base
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-mir-navigation
sudo apt install ros-noetic-mir-gazebo
sudo apt install ros-noetic-ira-laser-tools


to create the project do this
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


do this in every terminal
. devel/setup.bash
