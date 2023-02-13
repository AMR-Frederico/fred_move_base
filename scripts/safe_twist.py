#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
# from sensor_msgs import range
# from sensor_msgs.msg import Joy
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
controler_buttons = {"square": None,
                     "circle": None,
                     "triangule": None,
                     "x": None,
                     "R2": None,
                     "L2": None}
abort = False
pressed = True
distance_detected = 1000


def abort_move():
    global abort
    abort = True
    safe_cmd_vel_msg.linear.x = 0
    safe_cmd_vel_msg.angular.z = 0
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)


def callbackSensor(msg):
    global distance_detected
    distance_detected = msg.data

    # def callbackSensor(msg):
    # distanceSensor[0] = msg.range


def joy_callback(msg):

    global abort
    global pressed

    controler_buttons["circle"] = msg.buttons[2]
    controler_buttons["square"] = msg.buttons[0]
    controler_buttons["triangule"] = msg.buttons[3]
    controler_buttons["x"] = msg.buttons[1]
    controler_buttons["L2"] = msg.buttons[6]
    controler_buttons["R2"] = msg.buttons[7]
    controler_buttons["L1"] = msg.buttons[5]
    controler_buttons["R1"] = msg.buttons[4]

    if(controler_buttons["square"]):
        ...
    if(controler_buttons["x"]):
        abort_move()
    if(controler_buttons["triangule"]):
        # print("led_on")
        led_strip_pub.publish(1)
    if(controler_buttons["R1"]):
        led_strip_pub.publish(2)
    if(controler_buttons["L1"]):
        led_strip_pub.publish(0)
    if(controler_buttons["L2"] and controler_buttons["R2"]):
        print("CLEAR----------------")
        abort = False


def callbackCmdVel(msg):
    global abort
    global distance_detected
    print(abort)
    # CHECA se tem algo na frente dele

    if(not abort):
        print(distance_detected)

        # KP = distance_detected/300
        KP = 1
        if(KP > 1):
            KP = 1
        if(distance_detected <= 20):
            KP = 0
        safe_cmd_vel_msg.linear.x = msg.linear.x*KP
        safe_cmd_vel_msg.angular.z = msg.angular.z*KP
        # Se não repassa o cmd_vel
        # print("safe")

    else:
        # Se tiver para o robo
        safe_cmd_vel_msg.linear.x = 0
        safe_cmd_vel_msg.angular.z = 0
        print("danger")
    print(f"X { safe_cmd_vel_msg.linear.x} Z: {safe_cmd_vel_msg.angular.z}")
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_safe')
    rospy.Subscriber("/cmd_vel", Twist, callbackCmdVel)
    rospy.Subscriber('sensor/ultrasonic/left/distance', Int32, callbackSensor)
    rospy.Subscriber('sensor/ultrasonic/middle/distance',
                     Int32, callbackSensor)
    rospy.Subscriber('sensor/ultrasonic/right/distance', Int32, callbackSensor)
    # rospy.Subscriber('/joy', Joy, joy_callback)

    while not rospy.is_shutdown():

        rospy.spin()
# print(start)