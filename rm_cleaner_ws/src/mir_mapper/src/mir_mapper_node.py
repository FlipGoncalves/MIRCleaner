#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import subprocess

def start_explore_lite():
    # Start explore_lite node for autonomous exploration
    rospy.loginfo("Starting explore_lite for autonomous exploration...")
    subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

def start_slam():
    # Start gmapping for SLAM
    rospy.loginfo("Starting gmapping for SLAM...")
    subprocess.Popen(["rosrun", "gmapping", "slam_gmapping", "scan:=scan_multi", "_map_update_interval:=3", "_linearUpdate:=0.5"])

def main():
    rospy.init_node('mir_mapper_node', anonymous=True)
    rospy.loginfo("Starting MIR Mapper Node...")

    # Start SLAM and exploration
    start_slam()
    rospy.sleep(5)  # Wait for SLAM to initialize
    start_explore_lite()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
