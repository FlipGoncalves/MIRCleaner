#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MIRCleaner:
    def __init__(self):
        rospy.init_node('mir_cleaner_node', anonymous=True)

        # Parameters
        self.cleaning_path = []
        self.map_data = None
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        rospy.loginfo("MIR Cleaner Node Initialized")

    def map_callback(self, data):
        self.map_data = data
        rospy.loginfo("Map received")
        self.generate_cleaning_path()

    def generate_cleaning_path(self):
        # Assuming the map is received as an OccupancyGrid
        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))
        # Create waypoints in a grid pattern to cover the map
        for y in range(0, self.map_data.info.height, 10):
            for x in range(0, self.map_data.info.width, 10):
                if map_array[y][x] == 0:  # Only navigate to free spaces
                    x_world = x * self.map_data.info.resolution + self.map_data.info.origin.position.x
                    y_world = y * self.map_data.info.resolution + self.map_data.info.origin.position.y
                    self.cleaning_path.append((x_world, y_world))
        
        rospy.loginfo("Cleaning path generated with {} waypoints".format(len(self.cleaning_path)))
        self.start_cleaning()

    def start_cleaning(self):
        for waypoint in self.cleaning_path:
            if rospy.is_shutdown():
                break
            self.move_to_goal(waypoint[0], waypoint[1])

    def move_to_goal(self, x_goal, y_goal):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Moving to goal: ({}, {})".format(x_goal, y_goal))
        self.client.send_goal(goal)
        wait = self.client.wait_for_result()

        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            result = self.client.get_result()
            rospy.loginfo("Reached goal: ({}, {})".format(x_goal, y_goal))

if __name__ == '__main__':
    try:
        cleaner = MIRCleaner()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
