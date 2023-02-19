#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int16, Bool


#TODO add joy drift if -10>x>10 ignore command

cmd_vel_msg = Twist()
vel_linear = 0
vel_angular = 0
manual_mode = True

DRIFT_ANALOG_TOLERANCE = 20

reset_odom = False
last_reset_odom = False

switch_mode = False
last_switch_mode = False

controler_buttons = {"square": None,
                     "circle": None,
                     "triangule": None,
                      "x": None,
                     "R2": None,
                     "L2": None,
                     "R_Y":0,
                     "R_X":0}


MAX_SPEED_ROBOT_LINEAR = 10 
MAX_SPEED_ROBOT_ANGULAR = 0.2
MAX_VALUE_CONTROLER = 127


# PUBS ---------------------------------
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def rising_edge(last,current):
    return current > last 

def call_change_mode(msg):
    global switch_mode 
    switch_mode = msg.data

def call_reset_odom(msg):
    global reset_odom  
    reset_odom = msg.data


def call_manual(msg):
    global manual_mode
    manual_mode = msg.data

def call_linear(msg):
    global controler_buttons
    if(abs(msg.data) > DRIFT_ANALOG_TOLERANCE):
         controler_buttons["R_X"] = msg.data
    else:
         controler_buttons["R_X"] = 0

def call_angular(msg):
    global controler_buttons
    if(abs(msg.data) > DRIFT_ANALOG_TOLERANCE):
        controler_buttons["R_Y"] = msg.data
    else:
        controler_buttons["R_Y"] = 0

def call_break(msg):
    global controler_buttons
    controler_buttons["x"] = msg.data


if __name__ == '__main__':
    rospy.init_node("joy_esp_interface_node")

    rospy.Subscriber("joy/controler/ps4/cmd_vel/linear", Int16, call_linear)
    rospy.Subscriber("joy/controler/ps4/cmd_vel/angular",
                     Int16, call_angular)
    rospy.Subscriber('joy/controler/ps4/break', Int16, call_break)

    rospy.Subscriber("/machine_state/control_mode/manual", Bool,call_manual )

    rospy.Subscriber("joy/controler/ps4/circle",Int16,call_reset_odom)

    rospy.Subscriber("/joy/controler/ps4/triangle",Int16,call_change_mode)

    sub_odom_reset = rospy.Publisher("odom/reset",Bool, queue_size = 1 )
    sub_change_mode = rospy.Publisher("/machine_state/control_mode/switch",Bool,queue_size = 1)


    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #only send comands if manual mode on 
        
        vel_angular = controler_buttons["R_Y"]*(MAX_SPEED_ROBOT_ANGULAR/MAX_VALUE_CONTROLER) #regra de tres equivalendo a velocidade maxima do controle com a do robo 
        vel_linear = controler_buttons["R_X"]*(MAX_SPEED_ROBOT_LINEAR/MAX_VALUE_CONTROLER)

        cmd_vel_msg.linear.x = vel_linear
        cmd_vel_msg.angular.z = vel_angular

        if(manual_mode):
            cmd_vel_pub.publish(cmd_vel_msg)
        # print(f"Manual: {manual_mode}| Linear: {vel_linear}| Angular: {vel_angular}")

        #circle
        odom_reset = rising_edge(last_reset_odom,reset_odom)
        last_reset_odom = reset_odom
        sub_odom_reset.publish(odom_reset)


        #triangle
        mode_change = rising_edge(last_switch_mode,switch_mode)
        last_switch_mode = switch_mode
        sub_change_mode.publish(mode_change)


        rate.sleep()
