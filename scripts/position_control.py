# /user/bin/env python3

import rospy
import PositionController

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

lin
linear = PositionController(1,1,1)


from time import time

setPoint_x = 0
setPoint_y = 0

tolerance_x = 0.1
tolerance_y = 0.1 

active_pid = False

Kp_linear = 0.35 
Ki_linear = 0
Kd_linear = 0

Kp_angular = 0
Ki_angular = 0
Kd_angular = 0

def proporcional(error, Kp): 
    return Kp*error

def integrative(error, Ki): 
    return 

def output()
    




def turn_on_controller_callback(msg):
    global active_pid
    active_pid = msg.data

def position_callback(position_msg):
    global setPoint_x
    setPoint_x = position_msg.data

def odom_callback(odom_msg):
    global desired_position, tolerance, active_pid

    current_position = odom_msg.pose.pose.position.x
    error = desired_position - current_position
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    if abs(error) < tolerance:
        vel_msg.linear.x = 0.0
    else:
        global Kp
        global Ki 
        global Kd 

       vel_msg.linear.x = linear.output()
       
    if(active_pid):   
        pub.publish(vel_msg)


def control_position():
    rospy.init_node('position_controller', anonymous=True)
    rospy.Subscriber("/control/position/x", Float64, position_callback)
    rospy.Subscriber("/control/on",Bool,turn_on_controller_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        control_position()
    except rospy.ROSInterruptException:
        pass



