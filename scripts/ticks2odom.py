#!/usr/bin/env python3
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32,Bool

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

# Parameters
wheeltrack = 0.300  # distance between whells
wheelradius = 0.075  # radius of the wheel in meters
TPR = 2400*3  # ticks per turn
left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0
heading = 0
reset_odom = False

x = 0.0
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

global zero_ref_th
zero_ref_th = 0 

def reset_callback(msg):
    global reset_odom
    reset_odom = msg.data

def leftTicksCallback(msg):
    global left_ticks
    left_ticks = msg.data


def rightTicksCallback(msg):
    global right_ticks
    right_ticks = msg.data


def headingCB(msg):
    global heading
    heading = msg.data


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
left_ticks_sub = rospy.Subscriber(
    "power/status/distance/ticks/left", Float32, leftTicksCallback)
right_ticks_sub = rospy.Subscriber(
    "power/status/distance/ticks/right", Float32, rightTicksCallback)
heading_sub = rospy.Subscriber("sensor/imu/yaw", Float32, headingCB)
odom_broadcaster = tf.TransformBroadcaster()

reset_odom_sub = rospy.Subscriber("/odom/reset",Bool,reset_callback)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()
    # print(left_ticks, right_ticks)

    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks
    dl = 2 * pi * wheelradius * delta_L / TPR
    dr = 2 * pi * wheelradius * delta_R / TPR
    dc = (dl + dr) / 2
    dt = (current_time - last_time).to_sec()
    dth = (dr-dl)/wheeltrack

    if dr == dl:
        dx = dr*cos(th)
        dy = dr*sin(th)

    else:
        radius = dc/dth

        iccX = x-radius*sin(th)
        iccY = y+radius*cos(th)

        dx = cos(dth) * (x-iccX) - sin(dth) * (y-iccY) + iccX - x
        dy = sin(dth) * (x-iccX) + cos(dth) * (y-iccY) + iccY - y

    x += dx
    y += dy
    #th = (th+dth) % (2*pi)
    th = heading - zero_ref_th
    print(zero_ref_th)
    if(reset_odom):

        zero_ref_th = th
        #th = 0
        x = 0 
        y = 0
        

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    if dt > 0:
        vx = dx/dt
        vy = dy/dt
        vth = dth/dt

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)

    last_left_ticks = left_ticks
    last_right_ticks = right_ticks
    last_time = current_time
    print(f'X:{x} | Y:{y} | Theta:{th}')
    r.sleep()