# -*- coding: utf-8 -*-

# /user/bin/env python3


import rospy
import tf
import math 

from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Bool 

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from time import time 

# ------------------------------- class pid 

class PositionController: 
    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KD = KD 
        self.KI = KI

        self.time = time()
        self.last_time = time()
        self.delta_time = 0

        self.error = 0
        self.last_error = 0
        self.delta_error = 0

        self.integral = 0
    
    def proporcional(self):

        return self.KP * self.error
    
    def integrative(self): 

        self.integral += self.error * self.delta_time
            #anti wind-up
        if(self.integral > 1.5 or self.integral < -1.5):
            self.integral = 0
        # print(self.integral)
        return self.integral * self.KI

    def derivative(self):
        self.delta_error = self.error - self.last_error

        if(self.delta_error != 0):
            self.delta_error = self.delta_error/self.delta_time
        else:
            self.delta_error = 0
        return self.delta_error*self.KD
            
    def output(self, kp, ki, kd, error):
        self.KP = kp
        self.KI = ki
        self.KD = kd

        self.error = error

        self.time = time()
        self.delta_time = self.time - self.last_time

        if (self.error != 0):
            output = self.proporcional() + self.integrative() + self.derivative()
        else: 
            output = self.proporcional() + self.derivative()
        
        self.last_error = self.error
        self.last_time = self.time

        return output

## ------------------------------------ end of class


## ----- constants pid controller -> linear 
KP_linear = 1#0.25
KI_linear = 0#0.01
KD_linear = 0#0

## ----- constants pid controller -> angular 
KP_angular = 0.5#6.3 #0.5
KI_angular = 0.3#1
KD_angular = 0.5#0

angular = PositionController(KP_angular, KI_angular, KD_angular)   # (p, i, d) correction in y axis
linear = PositionController(KP_linear, KI_linear, KD_linear)    # (p, i, d) correction in x axis

setpoint_x = 1
currentPosition_x = 0
Tolerance_x = 0.2   # value in meters 

setpoint_y = -1      # keep the robot on a straight line
currentPosition_y = 0

setpoint_theta = 0
currentTheta = 0
Tolerance_theta = math.pi/10   # value in rad 

active_pid = True

delta_x = 0
delta_y = 0

### -------- TOPICS
error_angular_msg = Float32()
error_linear_msg = Float32()

### -------- PUBLISHERS 
error_angular_pub = rospy.Publisher("/control/position/debug/angular/error", Float32, queue_size = 10)
error_linear_pub = rospy.Publisher("control/position/debug/linear/error", Float32, queue_size = 10)

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()

def turn_on_controller_callback(msg):
    global active_pid
    active_pid = msg.data

def setpoint_callback(msg):
    global setpoint_x, setpoint_y, setpoint_theta
    setpoint_x = msg.x
    setpoint_y = msg.y
    setpoint_theta = msg.theta

def kp_linear_callback(msg):
    global KP_linear
    KP_linear = msg.data

def ki_linear_callback(msg):
    global KI_linear 
    KI_linear = msg.data

def kd_linear_callback(msg): 
    global KD_linear
    KD_linear = msg.data

def kp_angular_callback(msg):
    global KP_angular 
    KP_angular = msg.data

def kd_angular_callback(msg):
    global KI_angular 
    KI_angular = msg.data 

def ki_angular_callback(msg):
    global KD_angular 
    KD_angular = msg.data

def odom_callback(odom_msg):
    global currentPosition_x, currentPosition_y, currentTheta
    global setpoint_x, setpoint_y, setpoint_theta
    global Tolerance_x, Tolerance_theta
    
    # pega a posição e orientação pelo /odom
    currentPosition_x = odom_msg.pose.pose.position.x
    currentPosition_y = odom_msg.pose.pose.position.y
    quarternion = [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w]
    
    # conversão quaternion para euler
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
    currentTheta = yaw
    
    delta_x = setpoint_x - currentPosition_x
    delta_y = setpoint_y - currentPosition_y
    
    # erro linear
    error_linear = math.hypot(delta_x,delta_y)
    if (currentPosition_x > setpoint_x):  # se o robô passar o setpoint em x, consequentimente deve ir para tras
        error_linear = - error_linear

    # erro angular
    setpoint_theta = math.atan2(delta_y, delta_x)
    error_angular = setpoint_theta - currentTheta

    if (delta_x < 0):
        error_linear = (-1)*error_linear

    #  condição para complementar o angulo caso necessário
    if (error_angular > math.pi):
        error_angular -= 2*math.pi
    elif (error_angular < -math.pi):
        error_angular += 2*math.pi

    # print(f"error linear {error_linear}")
    # print(f"error angular {error_angular}")
    
    error_angular_msg.data = error_angular
    error_linear_msg.data = error_linear

    error_angular_pub.publish(error_angular_msg)
    error_linear_pub.publish(error_linear_msg)

    if (abs(error_linear) < Tolerance_x) and (abs(error_angular) < Tolerance_theta):
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0 
        print("no objetivo")
    else:

       vel_msg.linear.x = linear.output(KP_linear, KI_linear, KD_linear, error_linear)
       vel_msg.angular.z = angular.output(KP_angular, KI_angular, KD_angular, error_angular)
    
    # print(f"velocidade linear {vel_msg.linear.x}")
    # print(f"velocidade angular {vel_msg.angular.z}")
    print(f"erro angular: {round(error_angular,3)} | error linear: {round(error_linear,3)} | vel_linear: {round(vel_msg.linear.x,3)} | vel_angular: {round(vel_msg.angular.z,0)}")

    if(active_pid):   
        cmd_vel_pub.publish(vel_msg)
        print("publicando velocidade")



def control_position():
    rospy.init_node('position_controller', anonymous=True)

    rospy.Subscriber("/control/on",Bool,turn_on_controller_callback)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/control/position/setup/goal", Pose2D, setpoint_callback)

    rospy.Subscriber("/control/position/setup/linear/kp", Float64, kp_linear_callback)
    rospy.Subscriber("/control/position/setup/linear/ki", Float64, ki_linear_callback)
    rospy.Subscriber("/control/position/setup/linear/kd", Float64, kd_linear_callback)

    rospy.Subscriber("/control/position/setup/angular/kp", Float64, kp_angular_callback)
    rospy.Subscriber("/control/position/setup/angular/ki", Float64, kd_angular_callback)
    rospy.Subscriber("/control/position/setup/angular/kd", Float64, ki_angular_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        control_position()
    except rospy.ROSInterruptException:
        pass



