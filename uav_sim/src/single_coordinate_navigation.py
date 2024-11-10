#!/usr/bin/env python3
'''

This Node Takes User Inputs in the form as
rosrun uav_sim  single_coordinate_navigation.py a1
# a1 is rack name you can provide , b3,c4,a4 etc.
Understanding Argument :
    a1,                                 a2                                 a3

   (x_values[0],y_values[0])   (x_values[0],y_values[1]) (x_values[0],y_values[2])

'''

import re
import sys
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction


def movebase_client():
    argv = sys.argv[1:]
    x_values = [7.5, 15, 24.5]
    y_values = [11.5, 16.2, 20.6, 25.1]

    input_segment = re.findall(r"[^\W\d_]+|\d+", argv[0])
    rack_row = input_segment[0]
    rack_column = input_segment[1]
    row_names = "abc"
    if rack_row in row_names:
        goal_x_loc = x_values[row_names.index(rack_row)]
    goal_y_loc = y_values[int(float(rack_column)-1)]
    print("Setting Goal Location\nRow : ", rack_row ," Column :",rack_column)
    print("Cordinates : x = ", goal_x_loc," y = ",goal_y_loc)
    print("-"*20)
    client= actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    print("Robot moving towards Goal")

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = goal_x_loc
    goal.target_pose.pose.position.y = goal_y_loc
    goal.target_pose.pose.orientation.z = 2.0

    client.send_goal(goal)
    wait=client.wait_for_result()
    if not wait:
        rospy.logerr("Action server Down !! ")
    else:
        print("Robot has Reached Rack ", argv[0])
        print("-"*20)


if __name__ == '__main__':
    try:
        rospy.init_node("MovebaseClient")
        move_robot = movebase_client()
        if (move_robot):
            rospy.loginfo("Goal executed :) ")
    except:
        rospy.ROSInterruptException
