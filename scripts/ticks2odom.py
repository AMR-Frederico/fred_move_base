#!/usr/bin/env python3
import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32,Bool
from sensor_msgs.msg import Imu
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

imu_quaternion = []

# x = 0
x = 0.24 #consider robot front  not base_link
y = 0.0
th = 0.0

vx = 0.0
vy = 0.0
vth = 0.0

heading_offset = 0.0 #offset para zerar o mpu 

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
    global imu_quaternion

    imu_quaternion = msg.orientation
    heading = tf.transformations.euler_from_quaternion([imu_quaternion.x, imu_quaternion.y, imu_quaternion.z, imu_quaternion.w])[2]


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
left_ticks_sub = rospy.Subscriber(
    "power/status/distance/ticks/left", Float32, leftTicksCallback)
right_ticks_sub = rospy.Subscriber(
    "power/status/distance/ticks/right", Float32, rightTicksCallback)
heading_sub = rospy.Subscriber("sensor/orientation/imu", Imu, headingCB)

reset_odom_sub = rospy.Subscriber("/odom/reset",Bool,reset_callback)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(50)

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
    # th = (th+dth) % (2*pi)
    th = heading - heading_offset

    if(reset_odom):
        # x = 0
        x = 0.24
        y = 0
        #th = 0
        heading_offset = heading

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # compute the odometry relative to the footprint frame
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom_broadcaster = tf.TransformBroadcaster()
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_footprint",
        "odom"
    )

    # crate a frame between base_link e base_footprint
    base_link_quat = tf.transformations.quaternion_from_euler(0, 0, 0)  # no rotation
    base_link_broadcaster = tf.TransformBroadcaster()
    base_link_broadcaster.sendTransform(
        (0, 0, 0.08),  # offset between base_footprint and base_link in meters
        base_link_quat,
        current_time,
        "base_link",
        "base_footprint"
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

    odom.child_frame_id = "base_footprint"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)

    last_left_ticks = left_ticks
    last_right_ticks = right_ticks
    last_time = current_time
    print(f'X:{x} | Y:{y} | Theta:{th}')
    r.sleep()