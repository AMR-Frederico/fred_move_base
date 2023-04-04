#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from  numpy import log 


# TOPICOS
safe_cmd_vel_msg = Twist()
safety_stop_msg = Bool()
cmd_vel = Twist()

# PUBS ---------------------------------
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)
safety_stop_pub = rospy.Publisher('/safety/emergency/stop', Bool, queue_size=10)

distanceSensor = []
safeValue = 200

abort = True
abort_status = False
last_abort_status = False

safe_ultrasonic_distance = 80

K = -4

distance_detected_left = 0
distance_detected_back = 0
distance_detected_right = 0

vel_linear = 0
vel_angular = 0

led_sinalization = False
led_sinalization_status = False
led_sinalization_last_status = False

MAX_SPEED_LINEAR = 1.5 
MAX_SPEED_ANGULAR = 20 


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
    distance_detected_left = msg.range
    #print(distance_detected_left)


def callbackSensorBack(msg):
    global distance_detected_back
    distance_detected_back = msg.range
    #print(distance_detected_back)


def callbackSensorRight(msg):
    global distance_detected_right
    distance_detected_right = msg.range
    #print(distance_detected_right)


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
    rate = rospy.Rate(100)

    rospy.Subscriber("/cmd_vel", Twist, callbackCmdVel)
    rospy.Subscriber('joy/controler/ps4/break', Int16, callbackAbortMove)
    rospy.Subscriber('odom', Odometry, callbackOdometry)

    rospy.Subscriber('sensor/range/ultrasonic/left', Range, callbackSensorLeft)
    rospy.Subscriber('sensor/range/ultrasonic/right', Range, callbackSensorRight)
    rospy.Subscriber('sensor/range/ultrasonic/back', Range, callbackSensorBack)
    rospy.Subscriber('safety/ultrasonic/distance', Int32, callbackSafeDistance)
    
    while not rospy.is_shutdown():
      
    # CHECA se tem algo na frente dele

        smalest_reading = 500 #max leitor reading = 357 [cm] -> each loop 
        safe = not abort

        readings_sonar = [distance_detected_left,distance_detected_back ,distance_detected_right]

        for i in range(0,2):
            if(readings_sonar[i]<smalest_reading):
                smalest_reading = readings_sonar[i]


        
        safety_corretion = smalest_reading/(2*safe_ultrasonic_distance)

        

        if(safety_corretion > 1): 
            safety_corretion= 1

        if(safety_corretion < 0.5):

            safety_corretion = 0


        danger_left = distance_detected_left <= safe_ultrasonic_distance
        danger_middle = distance_detected_back <= safe_ultrasonic_distance
        danger_right = distance_detected_right <= safe_ultrasonic_distance

        danger_distance = danger_left or danger_middle or danger_right
        safe_distance = not danger_distance

        if(safe): #comando do controle manual 
                
            # mais perto de um obstaculo do que a distancia segura 
            if(danger_distance):
                safe_cmd_vel_msg.linear.x =  K*cmd_vel.linear.x
                safe_cmd_vel_msg.angular.z = K*cmd_vel.angular.z
               
            
            if(safe_distance):
                safe_cmd_vel_msg.linear.x = (cmd_vel.linear.x)*safety_corretion
                safe_cmd_vel_msg.angular.z = (cmd_vel.angular.z)*safety_corretion
                
                
        if(abort):
            safe_cmd_vel_msg.linear.x =  K*cmd_vel.linear.x
            safe_cmd_vel_msg.angular.z =  K*cmd_vel.angular.z
            
        
        led_sinalization_status = abort or danger_distance

        #saturação da velocidade linear 
        if (safe_cmd_vel_msg.linear.x > MAX_SPEED_LINEAR):
            safe_cmd_vel_msg.linear.x = MAX_SPEED_LINEAR
        elif(safe_cmd_vel_msg.linear.x < -MAX_SPEED_LINEAR):
            safe_cmd_vel_msg.linear.x = -MAX_SPEED_LINEAR

        #saturação da velocidade angular 
        if (safe_cmd_vel_msg.angular.z > MAX_SPEED_ANGULAR):
            safe_cmd_vel_msg.angular.z = MAX_SPEED_ANGULAR
        elif(safe_cmd_vel_msg.angular.z < -MAX_SPEED_ANGULAR):
            safe_cmd_vel_msg.angular.z = -MAX_SPEED_ANGULAR


        if ((led_sinalization_status == False) and (led_sinalization_last_status == False)): 
            led_sinalization = False
           
        else: 
            led_sinalization = True
            

        safety_stop_msg.data = led_sinalization
        led_sinalization_last_status = led_sinalization_status

        safety_stop_pub.publish(safety_stop_msg)
        safe_cmd_vel_pub.publish(safe_cmd_vel_msg)
        rate.sleep()

# print(start)