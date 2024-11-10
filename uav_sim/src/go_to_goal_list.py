#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from sensor_msgs.msg import Range
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
import rospy

vel_msg = Twist()
velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def sonar_callback(msg):
    global sonar
    sonar = msg.range
    #print(sonar)
sonar_sub = rospy.Subscriber('/sonar_height', Range, sonar_callback)
def move_to_goal(xGoal, yGoal):
    # define a client for to send goal requests to the move_base server through a SimpleActionClient
    ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # wait for the action server to come up
    while (not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
        rospy.loginfo("Waiting for the move_base action server to come up")

    goal = MoveBaseGoal()

    # set up the frame parameters
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # moving towards the goal*/

    goal.target_pose.pose.position = Point(xGoal, yGoal, 0)
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.2
    #goal.target_pose.pose.orientation.w = 0.0

    rospy.loginfo("Sending goal location ...")
    ac.send_goal(goal)

    ac.wait_for_result(rospy.Duration(60))#rospy.Duration(60)

    if (ac.get_state() == GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True

    else:
        rospy.loginfo("The robot failed to reach the destination")
        return False


if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    # x_goal = 5.89
    # y_goal = 12.51
    list_of_goals=[(6.89,12.51),(6.41,16.16),(6.43,20,61),(6.43,25.11)]
    list_of_goals_row2 = [(6.41,25.11) ,(6.41,20,79), (6.41,15.86), (6.89,11.49)]
    for i in range(len(list_of_goals)):
        print('start go to goal',i)
        x_goal=list_of_goals[i][0]
        y_goal=list_of_goals[i][1]
        move_to_goal(x_goal, y_goal)
        print('goal reached',i)
        # logic for sonar sensor to go to height
    while sonar < 5.0:
        vel_msg.linear.z = 2.0
        velocity_publisher.publish(vel_msg)

    else:
        vel_msg.linear.z = 0.0
        velocity_publisher.publish(vel_msg)
        print('height reached after goal nuber:',i)
        for i in range(len(list_of_goals_row2)):
            print('start go to goal second row', i)
            x_goal = list_of_goals_row2[i][0]
            y_goal = list_of_goals_row2[i][1]
            move_to_goal(x_goal, y_goal)
            print('goal reached second row', i)

        #rospy.spin()