#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

# Start explore_lite node for Autonomous Exploration
def start_explore_lite():
    rospy.loginfo("Starting explore_lite for autonomous exploration...")
    subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

# Start gmapping for SLAM
def start_slam():
    rospy.loginfo("Starting gmapping for SLAM...")
    subprocess.Popen(["rosrun", "gmapping", "slam_gmapping"])

# Main function
def main():
    # Start node
    rospy.init_node('mir_mapper_node', anonymous=True)
    rospy.loginfo("Starting MIR Mapper Node...")

    # Start SLAM and Exploration
    start_slam()
    rospy.sleep(5)  # Wait for SLAM to initialize
    start_explore_lite()

    rospy.spin()
    subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", "ExplorationMap"])
    rospy.loginfo("Saved map to ExplorationMap")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
