#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs import range
safe_cmd_vel_msg = Twist()
# TOPICOS


# PUBS ---------------------------------
safe_cmd_vel_pub = rospy.Publisher('/cmd_vel/safe', Twist, queue_size=10)


distanceSensor = []
safeValue = 200


def sensorSafe():

    return (distanceSensor[0] > safeValue)


def callbackSensor(msg):
    distanceSensor[0] = msg.range


def callbackCmdVel(msg):

    # CHECA se tem algo na frente dele
    if(sensorSafe):
        # Se não repassa o cmd_vel
        safe_cmd_vel_msg.linear.x = msg.linear.x
        safe_cmd_vel_msg.angular.z = msg.angular.z
    else:
        # Se tiver para o robo
        safe_cmd_vel_msg.linear.x = 0
        safe_cmd_vel_msg.angular.z = 0
    safe_cmd_vel_pub.publish(safe_cmd_vel_msg)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_safe')
    rospy.Subscriber("/cmd_vel", Twist, callbackCmdVel)
    rospy.Subscriber('/status/ultraSonic', range, callbackSensor)

    while not rospy.is_shutdown():
        rospy.spin()
