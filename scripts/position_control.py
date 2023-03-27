# /user/bin/env python3

import rospy


from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import Bool 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import time 
import math 

## ------------------------------- class pid 

class PositionController: 
    def __init__(self, KP, KI, KD,start_up):
        self.KP = KP
        self.KD = KD 
        self.KI = KI

        self.START_UP_ERROR = 0.3

        self.time = time()
        self.last_time = time()
        self.delta_time = 0

        self.error = 0
        self.last_error = 0
        self.delta_error = 0

        self.integral = 0
        self.start_up = start_up
    
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

        if(abs(self.error) < self.START_UP_ERROR):
            if(self.start_up!=0):
                if(output > start_up):
                    return output
                else:   
                    return start_up
            


        return output

## ------------------------------------ end of class


## ----- constants pid controller -> linear 
KP_linear = 0.35 #0.35
KI_linear = 0.05
KD_linear = 0

## ----- constants pid controller -> angular 
KP_angular = 20   #6.3
KI_angular = 1     #1
KD_angular = 0

angular = PositionController(KP_angular, KI_angular, KD_angular,0)   # (p, i, d) correction in y axis
linear = PositionController(KP_linear, KI_linear, KD_linear,0)    # (p, i, d) correction in x axis

setPoint_x = 0
currentPosition_x = 0
Tolerance_x = 0.2   # value in meters 

setPoint_y = 0      # keep the robot on a straight line
currentPosition_y = 0
Tolerance_y = 0.2   # value in meters 

currentTheta = 0

active_pid = True

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

def position_x_callback(position_msg):
    global setPoint_x
    setPoint_x = position_msg.data

def position_y_callback(position_msg):
    global setPoint_y
    setPoint_y = position_msg.data

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


def robot_yaw_callback(msg):
    global currentTheta
    #currentTheta = msg.data


def odom_callback(odom_msg):
    global setPoint_x, setPoint_y
    global currentPosition_x, currentPosition_y, currentTheta
    global Tolerance_x, Tolerance_y

    global active_pid

    # pega a posição pelo topico da odometria
    currentPosition_x = odom_msg.pose.pose.position.x
    currentPosition_y = odom_msg.pose.pose.position.y
    current_quat = odom_msg.pose.pose.orientation
    quat_list = [current_quat.x,current_quat.y,current_quat.z,current_quat.w]
    (roll,pitch,yaw) = euler_from_quaternion(quat_list)
    current_theta = yaw
    print(current_theta)


    delta_x = setPoint_x - currentPosition_x
    delta_y = setPoint_y - currentPosition_y

    error_linear = delta_x

    if(delta_x == 0) or (delta_y == 0):
        error_angular = 0
    else:
        # error_angular = math.atan(delta_y/delta_x) - currentTheta
        error_angular =  currentTheta

    # print(f"error linear {error_linear}")
    # print(f"error angular {error_angular}")
    
    error_angular_msg.data = error_angular
    error_linear_msg.data = error_linear

    error_angular_pub.publish(error_angular_msg)
    error_linear_pub.publish(error_linear_msg)

    if (abs(error_linear) < Tolerance_x) and (abs(error_angular) < Tolerance_y):
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0 
        print("no objetivo")
    else:

       vel_msg.linear.x = linear.output(KP_linear, KI_linear, KD_linear, error_linear)
       vel_msg.angular.z = angular.output(KP_angular, KI_angular, KD_angular, error_angular)
    
    
    # print(f"velocidade linear {vel_msg.linear.x}")
    # print(f"velocidade angular {vel_msg.angular.z}")
    print(f"erro angular: {error_angular} | error linear: {error_linear} | vel_linear: {vel_msg.linear.x} | vel_angular: {vel_msg.angular.z}")


    if(active_pid):   
        cmd_vel_pub.publish(vel_msg)



def control_position():
    rospy.init_node('position_controller', anonymous=True)

    rospy.Subscriber("/control/position/x", Float64, position_x_callback)
    rospy.Subscriber("/control/position/y", Float64, position_y_callback)
    rospy.Subscriber("/control/on",Bool,turn_on_controller_callback)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/sensor/imu/yaw", Float32, robot_yaw_callback)

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



