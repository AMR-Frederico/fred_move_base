#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
# from sensor_msgs import range
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
safe_cmd_vel_msg = Twist()
# TOPICOS
led_strip_msg = Float32()

# PUBS ---------------------------------
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)
led_strip_pub = rospy.Publisher("/cmd/led_strip/color", Float32, queue_size=10)

distanceSensor = []
safeValue = 200
controler_buttons = {"square": None,
                     "circle": None, "triangule": None, "x": None}
abort = False


def abort_move():
    global abort
    abort = True
    safe_cmd_vel_msg.linear.x = 0
    safe_cmd_vel_msg.angular.z = 0
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)


def sensorSafe():

    # return (distanceSensor[0] > safeValue)
    return True

    # def callbackSensor(msg):
    # distanceSensor[0] = msg.range


def joy_callback(msg):

    controler_buttons["circle"] = msg.buttons[2]
    controler_buttons["square"] = msg.buttons[0]
    controler_buttons["triangule"] = msg.buttons[3]
    controler_buttons["x"] = msg.buttons[1]

    if(controler_buttons["square"]):
        ...
    if(controler_buttons["x"]):
        abort_move()
    if(controler_buttons["triangule"]):
        led_strip_pub.publish(1)


def callbackCmdVel(msg):
    global abort
    print(abort)
    # CHECA se tem algo na frente dele

    if(sensorSafe and not abort):
        # Se não repassa o cmd_vel
        safe_cmd_vel_msg.linear.x = msg.linear.x
        safe_cmd_vel_msg.angular.z = msg.angular.z
        print("safe")
    else:
        # Se tiver para o robo
        safe_cmd_vel_msg.linear.x = 0
        safe_cmd_vel_msg.angular.z = 0
        print("danger")
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_safe')
    rospy.Subscriber("/cmd_vel", Twist, callbackCmdVel)
    # rospy.Subscriber('/status/ultraSonic', range, callbackSensor)
    rospy.Subscriber('/joy', Joy, joy_callback)

    while not rospy.is_shutdown():

        rospy.spin()
