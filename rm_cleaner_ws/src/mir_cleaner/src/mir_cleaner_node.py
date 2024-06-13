#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid

class CleaningAgent:
    def __init__(self, cleaning_percentage):
        self.cells_per_step = 20
        self.width = 0.64
        self.cleaning_percentage = cleaning_percentage
        self.cleaned_area = 0
        self.total_area = 0
        self.goal_area = 0
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def map_callback(self, data):
        self.map_data = data
        self.calculate_goal_free_cells()
        self.clean_map()

    def calculate_goal_free_cells(self):
        self.total_area = sum(1 for cell in self.map_data.data if cell == 0)
        self.goal_area = int(self.total_area * self.cleaning_percentage / 100)
        rospy.loginfo("Total free cells: %d, Goal free cells: %d", self.total_area, self.goal_area)

    def clean_map(self):
        resolution = self.map_data.info.resolution * self.cells_per_step
        max_height = int(abs(self.map_data.info.origin.position.y) / 10) - 1
        max_width = int(abs(self.map_data.info.origin.position.x) / 10) - 1

        for y in range(0, int(self.map_data.info.width / 10 / (self.width * 20)) + 1):
            for x in range(0, int(self.map_data.info.height / 10 / self.cells_per_step) + 1):
                zigzag = 1 if y % 2 == 0 else -1
                self.move_to_cell((-max_width + x * resolution)*zigzag, -max_height + y * self.width, zigzag)
                rospy.loginfo(f"\tTotal Area: {self.total_area}; Goal Area: {self.goal_area}; Cleaned Area: {self.cleaned_area}")

                if self.cleaned_area >= self.goal_area:
                    return

            if self.cleaned_area >= self.goal_area:
                break

        rospy.loginfo("Cleaning goal reached: %d cells cleaned", self.cleaned_area)

    def move_to_cell(self, x, y, ori):
        if self.cleaned_area >= self.goal_area:
            return

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.pose.orientation.z = 0

        if ori == -1:
            goal.target_pose.pose.orientation.w = 0
            goal.target_pose.pose.orientation.z = 1

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
        else:
            result = self.client.get_result()
            if result:
                self.cleaned_area += self.width / self.map_data.info.resolution * self.cells_per_step
                rospy.loginfo("Cleaned cell at: (%.2f, %.2f, %.2f)", x, y, ori)

if __name__ == '__main__':
    try:
        rospy.init_node('cleaning_agent', anonymous=True)
        cleaning_percentage = rospy.get_param('~percentage')
        agent = CleaningAgent(cleaning_percentage)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Cleaning agent node terminated.")