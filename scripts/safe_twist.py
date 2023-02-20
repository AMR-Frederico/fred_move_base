#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

# TOPICOS
safe_cmd_vel_msg = Twist()
safety_stop_msg = Bool()
cmd_vel = Twist()

# PUBS ---------------------------------
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)
safety_stop_pub = rospy.Publisher('/safety/emergency/stop', Bool, queue_size=10)

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

vel_linear = 0
vel_angular = 0

led_sinalization = False
led_sinalization_status = False
led_sinalization_last_status = False


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


def callbackOdometry(odom):
    global vel_linear
    vel_linear = odom.twist.twist.linear.x
    # global vel_angular = odom.twist.angular.y


def callbackCmdVel(msg):
    global cmd_vel
    cmd_vel = msg 
    

if __name__ == '__main__':
    rospy.init_node('cmd_vel_safe')
    rate = rospy.Rate(10)

    rospy.Subscriber("/cmd_vel", Twist, callbackCmdVel)
    rospy.Subscriber('joy/controler/ps4/break', Int16, callbackAbortMove)
    rospy.Subscriber('odom', Odometry, callbackOdometry)

    rospy.Subscriber('sensor/ultrasonic/left/distance', Int32, callbackSensorLeft)
    rospy.Subscriber('sensor/ultrasonic/middle/distance', Int32, callbackSensorMiddle)
    rospy.Subscriber('sensor/ultrasonic/right/distance', Int32, callbackSensorRight)
    rospy.Subscriber('safety/ultrasonic/distance', Int32, callbackSafeDistance)
    
    while not rospy.is_shutdown():
        #print(abort)
    # CHECA se tem algo na frente dele

        if(not abort):
            #print(distance_detected)

            KP = (distance_detected_left + distance_detected_middle + distance_detected_right)/3
            KP = 1   # 357cm -> alcance máximo do sensor
                        # a ideia é fazer um fator de correção de acordo com a proximidade do objeto
            if(KP > 1):
                KP = 1
                
            # safe distance vem do controle de posição
            if((distance_detected_left <= safe_ultrasonic_distance) or (distance_detected_middle <= safe_ultrasonic_distance) or (distance_detected_right <= safe_ultrasonic_distance)):
                safe_cmd_vel_msg.linear.x = 0
                safe_cmd_vel_msg.angular.z = 0
                led_sinalization_last_status = True

                print("DANGER -> obstacle ahead")
            
            else:
                safe_cmd_vel_msg.linear.x = cmd_vel.linear.x*KP
                safe_cmd_vel_msg.angular.z = cmd_vel.angular.z*KP
                led_sinalization_status = False

                print("safe")
        
        else: 
            safe_cmd_vel_msg.linear.x = 0
            safe_cmd_vel_msg.angular.z = 0
            print("DANGER -> joy requires stop")
            led_sinalization_status= True
        
        safe_cmd_vel_pub.publish(safe_cmd_vel_msg)

        if ((led_sinalization_status == False) and (led_sinalization_last_status == False)): 
            led_sinalization = False
            print(led_sinalization_status)
        else: 
            led_sinalization = True
            print("sinalization -> true")

        safety_stop_msg.data = led_sinalization
        safety_stop_pub.publish(safety_stop_msg)
        led_sinalization_last_status = led_sinalization_status

    
        rate.sleep()

# print(start)