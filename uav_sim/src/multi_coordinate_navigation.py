#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction

def movebase_client():

    client= actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    ### Multiple Coordinates
    goal_points =  [-3   , -12  ,
                    -3   , -12  ,
                    -4.5 , -12  ,
                    -6.6 , -9.7 ,
                    -2   , -12  ,
                    -1.67, -9.28,
                    1    , -11  ,
                    3.64 , -9.34,
                    4.3  , -10.5 ,
                    6.3  , -11.5 ,
                    8.6  , -8.89, ]


    goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    for i in range(0,20,2):
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_points[i]
        goal.target_pose.pose.position.y = goal_points[i+1]
        goal.target_pose.pose.orientation.z = -0.99

        client.send_goal(goal)
        wait=client.wait_for_result()
        if not wait:
            rospy.logerr("Action server Down !! ")
        else:
            print("A goal is Completed")
    return 1
if __name__ == '__main__':
    try:
        rospy.init_node("MovebaseClient")
        move_robot = movebase_client()
        if(move_robot):
            rospy.loginfo("Goal executed :) ")
    except:
        rospy.ROSInterruptException