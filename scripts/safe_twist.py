#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Int16, Bool, Float32
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

robot_vel = Twist()
cmd_vel = Twist()

robot_blockage = False 
blockage_frag = False
blockage_previous_frag = False

abort_command = True    # the robot starts blocked
abort_flag = False
abort_previous_flag = False

left_detection = 500    # max reading 
right_detection = 500
back_detection = 500

ultrasonic_measurements = []

# ------ publishers 
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)
safety_stop_pub = rospy.Publisher('/safety/emergency/stop', Bool, queue_size=10)
safety_distance_pub = rospy.Publisher('/safety/abort/distance', Bool, queue_size=10)

MIN_DIST_CLEARANCE = 80      # distance in centimeters 

MOTOR_BRAKE_FACTOR = -4

MAX_LINEAR_SPEED = 5
MAX_ANGULAR_SPEED = 10

def abort_callback(abort_msg): 
    global abort_command, abort_flag, abort_previous_flag

    abort_flag = abort_msg.data

    if (abort_flag and not abort_previous_flag):
        abort_command = not abort_command
    
    abort_previous_flag = abort_flag

    # rospy.loginfo("BOTAO DE ABORT PRESSIONADO\n")

def leftUltrasonic_callback(sensor_msg): 
    global left_detection

    left_detection = sensor_msg.data

def rightUltrasonic_callback(sensor_msg):
    global right_detection 

    right_detection = sensor_msg.data

def backUltrasonic_callback(sensor_msg):
    global back_detection

    back_detection = sensor_msg.data

def odom_callback(odom_msg): 
    global robot_vel

    robot_vel.linear.x = odom_msg.twist.twist.linear.x
    robot_vel.angular.z = odom_msg.twist.twist.angular.z

def cmdVel_callback(vel_msg):
    global cmd_vel 

    cmd_vel = vel_msg

    if abort_command or in_danger_zone: 
        cmd_vel.linear.x = 0
        cmd_vel.angular.z = 0 


if __name__ == '__main__':
    rospy.init_node('cmd_vel_safe')
    rate = rospy.Rate(50)

    rospy.Subscriber("/cmd_vel", Twist, cmdVel_callback)
    rospy.Subscriber('joy/controler/ps4/break', Int16, abort_callback)
    rospy.Subscriber('odom', Odometry, odom_callback)

    # rospy.Subscriber('sensor/range/ultrasonic/left', Float32, leftUltrasonic_callback)
    # rospy.Subscriber('sensor/range/ultrasonic/right', Float32, rightUltrasonic_callback)
    # rospy.Subscriber('sensor/range/ultrasonic/back', Float32, backUltrasonic_callback)
    
    while not rospy.is_shutdown():
        
        smallest_measurement = 500

        in_danger_zone = False 
        in_safe_zone = True   

        robot_safety = not abort_command

        ultrasonic_measurements = [left_detection, right_detection, back_detection]

        for i in range(0,2):
            if(ultrasonic_measurements[i] < smallest_measurement):
                smallest_measurement = ultrasonic_measurements[i]

        braking_factor = smallest_measurement/(2*MIN_DIST_CLEARANCE)

        if braking_factor > 1:
            braking_factor = 1
        
        if braking_factor < 0.5:
            braking_factor = 0 

        if (
            left_detection < MIN_DIST_CLEARANCE     or
            right_detection < MIN_DIST_CLEARANCE    or 
            back_detection < MIN_DIST_CLEARANCE 
        ):
        
            in_danger_zone = True

        in_safe_zone = not in_danger_zone

        rospy.loginfo(f"SAFE TWIST: Ultrassom -> esquerda: {left_detection} | direita: {right_detection} | back: {back_detection}")

        if robot_safety:
        
            if in_danger_zone:
                cmd_vel.linear.x = robot_vel.linear.x * MOTOR_BRAKE_FACTOR
                cmd_vel.angular.z = robot_vel.angular.z * MOTOR_BRAKE_FACTOR
                rospy.loginfo(f"SAFE TWIST: Robot in the danger zone, obstacle ahead!")

            if in_safe_zone:
                braking_factor = 1

                cmd_vel.linear.x = cmd_vel.linear.x * braking_factor
                cmd_vel.angular.z = cmd_vel.angular.z * braking_factor
                rospy.loginfo(f"SAFE TWIST: Robot in the safety zone")

        if abort_command:

            
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0

            # cmd_vel.linear.x = robot_vel.linear.x * MOTOR_BRAKE_FACTOR
            # cmd_vel.angular.z = robot_vel.angular.z * MOTOR_BRAKE_FACTOR

            rospy.loginfo(f"SAFE TWIST: --------------------- MANUAL SAFETY STOP ---------------------------------")

        if (cmd_vel.linear.x > MAX_LINEAR_SPEED):
            cmd_vel.linear.x = MAX_LINEAR_SPEED
    
        elif(cmd_vel.linear.x < -MAX_LINEAR_SPEED):
            cmd_vel.linear.x = -MAX_LINEAR_SPEED

        if (cmd_vel.angular.z > MAX_ANGULAR_SPEED):
            cmd_vel.angular.z = MAX_ANGULAR_SPEED
        
        elif(cmd_vel.angular.z < -MAX_ANGULAR_SPEED):
            cmd_vel.angular.z = -MAX_ANGULAR_SPEED

        safe_cmd_vel_pub.publish(cmd_vel)
        safety_stop_pub.publish(abort_command or in_danger_zone)
        safety_distance_pub.publish(in_danger_zone)

        rate.sleep()
