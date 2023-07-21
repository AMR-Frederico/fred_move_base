#!/usr/bin/env python3

import rospy 
from std_msgs.msg import Int16, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

robot_vel = Twist()
cmd_vel = Twist()

abort_command = True    # the robot starts blocked
abort_flag = False
abort_previous_flag = False

left_detection = 500    # max reading 
right_detection = 500
back_detection = 500

ultrasonic_measurements = []

robot_blockage = Bool()
blockage_frag = False
blockage_previous_frag = False

MIN_DIST_CLEARANCE = 80      # distance in centimeters 

MOTOR_BRAKE_FACTOR = -4

MAX_LINEAR_SPEED = 10
MAX_ANGULAR_SPEED = 10

# ------ publishers 
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)
safety_stop_pub = rospy.Publisher('/safety/emergency/stop', Bool, queue_size=10)
safety_distance_pub = rospy.Publisher('/safety/abort/distance', Bool, queue_size=10)



def cmdVel_callback(vel_msg):
    global cmd_vel 

    cmd_vel = vel_msg

    
    main()


def odom_callback(odom_msg): 
    global robot_vel

    robot_vel.linear.x = odom_msg.twist.twist.linear.x
    robot_vel.angular.z = odom_msg.twist.twist.angular.z
    

def abort_callback(abort_msg): 
    global abort_command, abort_flag, abort_previous_flag

    abort_flag = abort_msg.data

    if (abort_flag and not abort_previous_flag):
        abort_command = not abort_command
    
    abort_previous_flag = abort_flag


def leftUltrasonic_callback(sensor_msg): 
    global left_detection

    left_detection = sensor_msg.range

def rightUltrasonic_callback(sensor_msg):
    global right_detection 

    right_detection = sensor_msg.range

def backUltrasonic_callback(sensor_msg):
    global back_detection

    back_detection = sensor_msg.range

def main():
    global abort_command
    global ultrasonic_measurements, left_detection, right_detection, back_detection
    global robot_blockage, blockage_frag, blockage_previous_frag
    global cmd_vel, robot_vel
    
    smallest_measurement = 500

    in_danger_zone = False 
    in_safe_zone = False   

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

    rospy.loginfo(f"SAFE TWIST: esquerda: {left_detection} | direita: {right_detection} | back: {back_detection}")

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


    blockage_frag = abort_command or in_danger_zone
    
    if (not blockage_frag and not blockage_previous_frag):
        robot_blockage.data = False
    
    else:
        robot_blockage.data = True
    

    print(f"vel linear: {cmd_vel.linear.x}")


    rospy.loginfo(f"SAFE TWIST: Robot safety blockage  -> status: {robot_blockage.data}")
    blockage_previous_frag = blockage_frag

    safe_cmd_vel_pub.publish(cmd_vel)
    
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0

    safety_stop_pub.publish(robot_blockage)



if __name__ == '__main__':

    rospy.init_node('cmd_vel_safe')
    rate = rospy.Rate(50)

    # velocities 
    rospy.Subscriber('/cmd_vel', Twist, cmdVel_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # manual abort command 
    rospy.Subscriber('/joy/controler/ps4/break', Int16, abort_callback)

    # ultrasonic detection 
    rospy.Subscriber('/sensor/range/ultrasonic/left', Range, leftUltrasonic_callback)
    rospy.Subscriber('/sensor/range/ultrasonic/right', Range, rightUltrasonic_callback)
    rospy.Subscriber('/sensor/range/ultrasonic/back', Range, backUltrasonic_callback)

    while not rospy.is_shutdown():
        
        rate.sleep()
