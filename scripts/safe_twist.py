#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int16

safe_cmd_vel_msg = Twist()
# TOPICOS
led_strip_msg = Float32()

# PUBS ---------------------------------
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)
led_strip_pub = rospy.Publisher("/cmd/led_strip/color", Float32, queue_size=10)

distanceSensor = []
safeValue = 200

abort = False
abort_status = False
last_abort_status = False

safe_ultrasonic_distance = 20

distance_detected = 1000

distance_detected_left = 0
distance_detected_middle = 0
distance_detected_right = 0

def callbackAbortMove(msg):
    global last_abort_status
    global abort_status
    global abort
    abort_status = msg.data
    if(abort_status > last_abort_status): 
        abort = not abort

    last_abort_status = abort_status
    # print(abort)


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
    global safe_ultrasonic_distance
    safe_ultrasonic_distance = 20

def callbackSafeDistance(msg):
    global safe_ultrasonic_distance
    safe_ultrasonic_distance = msg.data


def callbackCmdVel(msg):
    global abort

    global distance_detected_left
    global distance_detected_middle
    global distance_detected_right

    global safe_ultrasonic_distance

    #print(abort)
    # CHECA se tem algo na frente dele

    if(not abort):
        #print(distance_detected)

        KP = (distance_detected_left + distance_detected_middle + distance_detected_right)/3
        KP = KP/300   # 357cm -> alcance máximo do sensor
                      # a ideia é fazer um fator de correção de acordo com a proximidade do objeto
        if(KP > 1):
            KP = 1
            
        # safe distance vem do controle de posição
        if((distance_detected_left <= safe_ultrasonic_distance) or (distance_detected_middle <= safe_ultrasonic_distance) or (distance_detected_right <= safe_ultrasonic_distance)):
            safe_cmd_vel_msg.linear.x = 0
            safe_cmd_vel_msg.angular.z = 0
            print("DANGER -> obstacle ahead")
        else:
            safe_cmd_vel_msg.linear.x = msg.linear.x*KP
            safe_cmd_vel_msg.angular.z = msg.angular.z*KP
            print("safe")
    
    else: 
        safe_cmd_vel_msg.linear.x = 0
        safe_cmd_vel_msg.angular.z = 0
        print("DANGER -> joy requests break")
        

    #print(f"X { safe_cmd_vel_msg.linear.x} Z: {safe_cmd_vel_msg.angular.z}")
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_safe')

    rospy.Subscriber("/cmd_vel", Twist, callbackCmdVel)

    rospy.Subscriber('sensor/ultrasonic/left/distance', Int32, callbackSensorLeft)
    rospy.Subscriber('sensor/ultrasonic/middle/distance', Int32, callbackSensorMiddle)
    rospy.Subscriber('sensor/ultrasonic/right/distance', Int32, callbackSensorRight)

    #rospy.Subscriber('safety/ultrasonic/distance', Int32, callbackSafeDistance)

    rospy.Subscriber('joy/controler/ps4/break', Int16, callbackAbortMove)

    while not rospy.is_shutdown():

        rospy.spin()
# print(start)