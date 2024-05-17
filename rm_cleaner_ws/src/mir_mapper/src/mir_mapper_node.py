#!/usr/bin/env python

import rospy
import subprocess
from std_srvs.srv import Empty
import actionlib
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class MIRMapper:
    def __init__(self):
        rospy.init_node('mir_mapper_node', anonymous=True)

        # Parameters
        self.exploration_completed = False

        # Subscribers
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)

        # Start SLAM and exploration
        self.start_slam()
        rospy.sleep(5)  # Wait for SLAM to initialize
        self.start_explore_lite()

        rospy.spin()

    def start_explore_lite(self):
        # Start explore_lite node for autonomous exploration
        rospy.loginfo("Starting explore_lite for autonomous exploration...")
        subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

    def start_slam(self):
        # Start gmapping for SLAM
        rospy.loginfo("Starting gmapping for SLAM...")
        subprocess.Popen(["rosrun", "gmapping", "slam_gmapping", "scan:=f_scan" ])

    def goal_status_callback(self, msg):
        # Check if there are any active goals
        active_goals = [status for status in msg.status_list if status.status in [1, 2]]
        if not active_goals and not self.exploration_completed:
            rospy.loginfo("Exploration appears to be complete")
            self.exploration_completed = True
            self.save_map()

    def save_map(self):
        rospy.loginfo("Saving map...")
        subprocess.Popen(["rosrun", "map_server", "map_saver", "-f", "ExplorationMap" ])


if __name__ == '__main__':
    try:
        MIRMapper()
    except rospy.ROSInterruptException:
        pass
