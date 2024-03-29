#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16, Bool, Float32
from enum import Enum, IntEnum
from geometry_msgs.msg import PoseStamped,Pose2D

GHOST = 1 
LED = 0

class Fred_color(IntEnum):
    white = 0
    blue = 100
     
    yellow = 5
    pink = 4 
    green = 3 
    orange = 6 
    red = 2 
    black = -1 


goals = [ [1, 0.00, LED], 
          [5.20, 0.00, GHOST], 
          [7.0, 0.00, LED]]

abort_distance = False
abort_manual = 1
main_state = -5
goal_reached = False 
last_goal_reached = False
last_abort_status = False

led_on = False 

LED_ON_TIME = rospy.Duration(1)
start_timer = rospy.Time(0)
led_goal_reached = False 

goal_pose = Pose2D()

def setpoint_callback(msg): 
    global goal_pose
    goal_pose.x = msg.pose.position.x
    goal_pose.y = msg.pose.position.y 
    goal_pose.theta = msg.pose.orientation.z
    # print(goal_pose)

def call_abort_distance(msg):
    global abort_distance 
    abort_distance = msg.data 

def call_abort_manual(msg):
    global abort_manual
    global last_abort_status
    global abort_status

    abort_status = msg.data

    if abort_status > last_abort_status: 
        abort_manual = not abort_manual

    last_abort_status = abort_status

def call_main_state(msg):
    global main_state
    main_state = msg.data 

def call_goal_reached_callback(msg):
    global start_timer
    global last_goal_reached
    global led_goal_reached
    global goal_reached

    goal_reached = msg.data 

    if goal_reached > last_goal_reached: 
        # rospy.loginfo("LED MANAGER: start ")
        start_timer = rospy.Time.now()
        led_goal_reached = True
        pub_goal_reached_captured.publish(True)
        
        if goal_pose.theta == 1:
            led_goal_reached = False

    last_goal_reached = goal_reached 

    # print(rospy.Time.now() - start_timer)

    if rospy.Time.now() - start_timer > LED_ON_TIME:

        # rospy.loginfo("LED MANAGER: OFF")
        led_goal_reached = False

    #if goal_pose.theta == 1 :
        #led_goal_reached = False 

    # pub acknowledge -> next goal current
    #pub_goal_reached_captured.publish(True)

def main():
        global goal_pose
        print(goal_pose.theta)

        led_color = Fred_color.pink

        if main_state == 50:
            led_color = Fred_color.blue

            if led_goal_reached:
                led_color = Fred_color.green
                  


            if abort_distance:
                led_color = Fred_color.orange

           
        if main_state == 0:
            led_color = Fred_color.white

            if led_goal_reached:
                 led_color = Fred_color.green
            if abort_distance:
                led_color = Fred_color.orange

           
       
        if(main_state == 2):
             led_color = Fred_color.red


        # print(f"|{led_color}|  ABORT: DISTANCE {abort_distance}, MANUAL {abort_manual}, STATE {main_state}, GOAL_REACHED {led_goal_reached}")

        pub_fita_led.publish(led_color)
        led_on = False 

if __name__ == '__main__':
    rospy.init_node('led_manager')
    rate = rospy.Rate(50)
    pub_fita_led = rospy.Publisher("/cmd/led_strip/color", Float32, queue_size=5)
    pub_goal_reached_captured = rospy.Publisher("/goal_manager/goal/reached/ack", Bool, queue_size=5)

    rospy.Subscriber("/goal_manager/goal/current", PoseStamped, setpoint_callback)

    rospy.Subscriber('/safety/abort/distance', Bool, call_abort_distance)
    rospy.Subscriber('/joy/controler/ps4/break', Int16, call_abort_manual)
    rospy.Subscriber('/machine_state/main', Int16, call_main_state)
    rospy.Subscriber("/goal_manager/goal/reached", Bool, call_goal_reached_callback)

    while not rospy.is_shutdown():
        main()
        rate.sleep()
