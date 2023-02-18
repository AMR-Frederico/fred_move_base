#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32

safe_cmd_vel_msg = Twist()
# TOPICOS
led_strip_msg = Float32()

# PUBS ---------------------------------
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)
led_strip_pub = rospy.Publisher("/cmd/led_strip/color", Float32, queue_size=10)

distanceSensor = []
safeValue = 200

abort = False
pressed = True
distance_detected = 1000

def abort_move():
    global abort
    abort = True
    safe_cmd_vel_msg.linear.x = 0
    safe_cmd_vel_msg.angular.z = 0
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)

def callbackSensorLeft(msg):
    global distance_detected_left
    distance_detected_left = msg.data
    #print(distance_detected_left)


def callbackSensorMiddle(msg):
    global distance_detected_middle
    distance_detected_middle = msg.data
    #print(distance_detected_middle)


def callbackSensorRight(msg):
    global distance_detected_right
    distance_detected_right = msg.data
    #print(distance_detected_right)

def callbackSafeDistance(msg):
    global safe_ultrasonic_distance
    safe_ultrasonic_distance = msg.data


def callbackCmdVel(msg):
    global abort

    #print(abort)
    # CHECA se tem algo na frente dele

    if(not abort):
        #print(distance_detected)
        global distance_detected_left
        global distance_detected_middle
        global distance_detected_right

        KP = (distance_detected_left + distance_detected_middle + distance_detected_right)/3
        KP = KP/200   # 357cm -> alcance máximo do sensor
                      # a ideia é fazer um fator de correção de acordo com a proximidade do objeto
        if(KP > 1):
            KP = 1
        if((distance_detected_left <= 20) or (distance_detected_middle <= 20) or (distance_detected_right <= 20)): # safe distance = 20cm 
            safe_cmd_vel_msg.linear.x = 0
            safe_cmd_vel_msg.angular.z = 0
            print("DANGER -> STOPPING ROBOT")
        else:
            safe_cmd_vel_msg.linear.x = msg.linear.x*KP
            safe_cmd_vel_msg.angular.z = msg.angular.z*KP
            print("safe")

    #print(f"X { safe_cmd_vel_msg.linear.x} Z: {safe_cmd_vel_msg.angular.z}")
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_safe')

    rospy.Subscriber("/cmd_vel", Twist, callbackCmdVel)

    rospy.Subscriber('sensor/ultrasonic/left/distance', Int32, callbackSensorLeft)
    rospy.Subscriber('sensor/ultrasonic/middle/distance', Int32, callbackSensorMiddle)
    rospy.Subscriber('sensor/ultrasonic/right/distance', Int32, callbackSensorRight)

    rospy.Subscriber('safety/ultrasonic/distance', Int32, callbackSafeDistance)

    while not rospy.is_shutdown():

        rospy.spin()
# print(start)