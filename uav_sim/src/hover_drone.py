#!/usr/bin/env python3
'''
This Node makes the drone fly
'''
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

desired_height = 1.0
velocity_msg=Twist()
def drone_flyer():
    global pub
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/sonar_height", Range, sonar_callback)
    rospy.init_node('fly_node', anonymous=True)

    rospy.spin()

def sonar_callback(msg):
    global velocity_msg , pub
    sonar_data=msg.range
    if(sonar_data>=desired_height):
        velocity_msg.linear.z=0.0
    else:
        velocity_msg.linear.z=0.5
        print("Reaching set point")
    pub.publish(velocity_msg)
    if(sonar_data>=desired_height):
        rospy.signal_shutdown("Reached the Point")

    print("Sonar Values : " , sonar_data)




if __name__ == '__main__':

    try:
        drone_flyer()
    except rospy.ROSInterruptException:
        pass