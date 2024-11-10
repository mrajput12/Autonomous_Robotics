#!/usr/bin/env python3
'''
This Node Scans a rack from left to right
Remaining :
    Add hieght chaning functionality

'''
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import LaserScan

class auto_scaning:

    def __init__(self):
        self.counter = 0
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sonar_sub = rospy.Subscriber("/sonar_height", Range, self.sonar_callback)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.send_control_commands)
        self._timer = rospy.Timer(rospy.Duration(0.05), self.send_control_commands)

        self.vel_msg = Twist()
        self.sonar_value = Range()
        self.lidar_value = LaserScan()
        self.regions = 0
        self.state = 0
        self.prev_state = 0
        self.height= [ 5.0, 1.5 ,3.5 , 5.5]


    def lidar_callback(self, msg):
        ## getting rid of infinity
        self.regions = {
        'left':   min(msg.ranges[0]   , 10),
        'front':  min(msg.ranges[180] , 10),
        'right':  min(msg.ranges[359] , 10),

    }

    def sonar_callback(self, msg):
        self.sonar_value = msg.range
        if self.regions['front'] == 10 and self.regions['right'] <= 7:
            if (self.sonar_value >= self.height[0]):
                self.vel_msg.linear.z = 0.0
            else:
                self.vel_msg.linear.z = 0.2

           # print('to check',self.sonar_value)

       # print(self.regions['right'])

    def send_control_commands(self,temp):

        if(self.regions['front'] == 10 and self.regions['right'] <=7  ):
            print("!! Completed !!  Right Side Scanning ")

            self.prev_state = self.state
            self.state = 2


        elif(self.regions['front'] == 10 and self.regions['left'] <=7  ):
            print("!! Completed !!  Left Side Scanning Completed")
            self.prev_state = 0
            self.state = 1

        elif(self.regions['right'] == 10 and self.regions['left'] ==10  ):
            print("Currently Scanning")
            self.state = 1

        if(self.state == 1 and self.prev_state == 0):
            print("Case Right")
            self.vel_msg.linear.y = -0.5
        elif(self.state == 2 and self.prev_state == 1):
            print("Case Left")
            self.vel_msg.linear.y = 0.0






































        self.velocity_pub.publish(self.vel_msg)








if __name__ == '__main__':
    rospy.init_node('rack_scanning_node')
    auto_scaning()



    rospy.spin()


# goals=[(5.56,11.54,-2.49),(5.72,15.55,-2.49),(6.26,19.93,-2.49)]
# #client_initialization
# rospy.init_node("auto_scanning")
# move_base_client=actionlib.SimpleActionClient('move_base',MoveBaseAction)
# #wait for action server to showup
# move_base_client.wait_for_server()
# for g in goals:
#     move_goal=MoveBaseGoal()
#     move_goal.target_pose.header.frame_id = "map"
#     move_goal.target_pose.header.stamp = rospy.Time.now()
#     move_goal.target_pose.pose.position.x=g[0]
#     move_goal.target_pose.pose.position.y = g[1]
#     move_goal.target_pose.pose.orientation.w = g[2]
#     move_base_client.send_goal(move_goal)
#     move_base_client.wait_for_result()


