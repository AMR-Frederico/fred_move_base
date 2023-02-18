#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int16, Bool


#TODO add joy drift if -10>x>10 ignore command

cmd_vel_msg = Twist()
vel_linear = 0
vel_angular = 0
manual_mode = True

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


def call_manual(msg):
    global manual_mode
    manual_mode = msg.data

def call_linear(msg):
    global controler_buttons
    controler_buttons["R_X"] = msg.data

def call_angular(msg):
    global controler_buttons
    controler_buttons["R_Y"] = msg.data

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

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        #only send comands if manual mode on 
        if(manual_mode):
            vel_angular = controler_buttons["R_Y"]*(MAX_SPEED_ROBOT_ANGULAR/MAX_VALUE_CONTROLER) #regra de tres equivalendo a velocidade maxima do controle com a do robo 
            vel_linear = controler_buttons["R_X"]*(MAX_SPEED_ROBOT_LINEAR/MAX_VALUE_CONTROLER)

        cmd_vel_msg.linear.x = vel_linear
        cmd_vel_msg.angular.z = vel_angular

        cmd_vel_pub.publish(cmd_vel_msg)
        print(f"Manual: {manual_mode}| Linear: {vel_linear}| Angular: {vel_angular}")
        rate.sleep()
