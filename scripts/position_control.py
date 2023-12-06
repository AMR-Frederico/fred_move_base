#!/usr/bin/env python3


from pid import PIDController

import math 
import rospy 
import tf2_ros
import tf

from tf.transformations import quaternion_multiply
from geometry_msgs.msg import Pose2D, PoseStamped,Quaternion, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool 

# flag da maquina de estados
active_pid = False

# pose atual do robo em relação a odometria
robot_pose = Pose2D()
odom_pose = Pose2D()
odom_quaternion = Quaternion()

# setpoint/goal 
goal_pose = Pose2D()
goal_pose.x = 0.25

# ------ publishers 
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)

# ------ messages 
cmd_vel = Twist()

# limites de velocidade 
MIN_VEL = 0.5     # velocidade para fazer curva 
MAX_VEL = 2

# ṔID angular setup 
KP_ANGULAR = 20
KI_ANGULAR = 1
KD_ANGULAR = 0

angular_vel = PIDController(KP_ANGULAR, KI_ANGULAR, KD_ANGULAR)

# variavel de controle de direção e sentido 
motion_direction = 1    #  1  --> orientação frontal 
                        # -1  --> orientação traseira

def turn_on_pid_callback(msg): 
    global active_pid
    active_pid = msg.data

def odom_callback(odom_msg): 
    global odom_pose, odom_quaternion

    odom_pose.x = odom_msg.pose.pose.position.x
    odom_pose.y = odom_msg.pose.pose.position.y
    odom_quaternion = odom_msg.pose.pose.orientation
    
    odom_pose.theta = tf.transformations.euler_from_quaternion([
        odom_quaternion.x, 
        odom_quaternion.y, 
        odom_quaternion.z, 
        odom_quaternion.w
        ])[2]

def setpoint_callback(goal_msg): 
    global goal_pose

    goal_pose.x = goal_msg.pose.position.x 
    goal_pose.y = goal_msg.pose.position.y 
    goal_pose.theta = tf.transformations.euler_from_quaternion([
        goal_msg.pose.orientation.x, 
        goal_msg.pose.orientation.y, 
        goal_msg.pose.orientation.z, 
        goal_msg.pose.orientation.w
        ])[2]
    
    # rospy.loginfo("POSITION CONTROL: Received new goal")

# Orientação e posição do robô com x+ apontado para trás e y+ para direita
def backward_orientation():
    global odom_pose, odom_quaternion
    bkward_pose = Pose2D()

    # * utilizando a odometria ------------------------------------------------------------------------------------------------------

    bkward_quaternion = Quaternion()
    bkward_pose.x = odom_pose.x
    bkward_pose.y = odom_pose.y
    
    q_rot = tf.transformations.quaternion_from_euler(0, 0, math.pi)

    bkward_quaternion = quaternion_multiply([odom_quaternion.x, odom_quaternion.y, odom_quaternion.z, odom_quaternion.w],q_rot)
    bkward_pose.theta = tf.transformations.euler_from_quaternion(bkward_quaternion)[2]

    # * ------------------------------------------------------------------------------------------------------------------------------

    # tf_buffer = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buffer)

    # try:
    #     transform = tf_buffer.lookup_transform("odom", "backward_orientation_link", rospy.Time(0), rospy.Duration(2.0))
        
    #     bkward_pose.x = transform.transform.translation.x
    #     bkward_pose.y = transform.transform.translation.y
    #     bkward_pose.theta = tf.transformations.euler_from_quaternion([
    #         transform.transform.rotation.x,
    #         transform.transform.rotation.y,
    #         transform.transform.rotation.z,
    #         transform.transform.rotation.w
    #     ])[2]
        
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rospy.logwarn("Failed to lookup transform from 'odom' to 'backward_orientation_link'")
    
    return bkward_pose

# Orientação e posição do robô com x+ apontado para frente e y+ para esquerda
def front_orientation():
    global odom_pose
    front_pose = Pose2D()

    # * utilizando a odometria ------------------------------------------------------------------------------------------------------

    front_pose.x = odom_pose.x
    front_pose.y = odom_pose.y
    front_pose.theta = odom_pose.theta

    # * ------------------------------------------------------------------------------------------------------------------------------

    # tf_buffer = tf2_ros.Buffer()
    # tf_listener = tf2_ros.TransformListener(tf_buffer)

    # try:
    #     transform = tf_buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(1.0))
        
    #     front_pose.x = transform.transform.translation.x
    #     front_pose.y = transform.transform.translation.y
    #     front_pose.theta = tf.transformations.euler_from_quaternion([
    #         transform.transform.rotation.x,
    #         transform.transform.rotation.y,
    #         transform.transform.rotation.z,
    #         transform.transform.rotation.w
    #     ])[2]
        
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     rospy.logwarn("Failed to lookup transform from 'odom' to 'base_link'")
    
    return front_pose

# reduz ângulo entre -2pi e 2pi
# def reduce_angle(angle):
#     if angle > 2*math.pi:
#         angle = angle - 4*math.pi
    
#     if angle < -2*math.pi:
#         angle = angle + 4*math.pi
    
#     return angle

# reduz ângulo entre -pi e pi
def reduce_angle(angle):
    while angle > math.pi:
        angle -= 2*math.pi
    
    while angle <= -math.pi:
        angle += 2*math.pi
    
    return angle

def position_control():
    global robot_pose, goal_pose
    global motion_direction, active_pid

    if motion_direction == 1: 
        robot_pose = front_orientation()
    
    elif motion_direction == -1:
        robot_pose = backward_orientation()
    
    dx = goal_pose.x - robot_pose.x 
    dy = goal_pose.y - robot_pose.y 

    error_angle = math.atan2(dy,dx)

    bkward_pose = backward_orientation()
    backward_orientation_error = reduce_angle(error_angle - bkward_pose.theta)

    front_pose = front_orientation()
    front_orientation_error = reduce_angle(error_angle - front_pose.theta)

    # print(f" front orientation error:  {front_orientation_error}")
    # print(f"backeward orientation error:  {backward_orientation_error}")

    # print(f"goal pose:  x = {goal_pose.x}   y = {goal_pose.y}")

    if (abs(front_orientation_error) > abs(backward_orientation_error)) and (motion_direction == 1):
        motion_direction = -1 
        robot_pose = backward_orientation()

        rospy.loginfo("POSITION CONTROL: Switching to backward orientation")


    elif (abs(backward_orientation_error) > abs(front_orientation_error)) and (motion_direction == -1): 
        motion_direction = 1 
        robot_pose = front_orientation()
        
        rospy.loginfo("POSITION CONTROL: Switching to front orientation")

    dx = goal_pose.x - robot_pose.x 
    dy = goal_pose.y - robot_pose.y 

    rospy.loginfo(f"POSITION: goal x = {goal_pose.x}  |  goal y = {goal_pose.y}")

    error_angle = math.atan2(dy,dx)

    orientation_error = reduce_angle(error_angle - robot_pose.theta)

    # # mapea a velocidade linear em função do erro de orientação, 
    # # se o erro for máximo -> vel_linear mínima
    # # sem o erro for mínimo -> vel_linear máxima

    rospy.loginfo(f"POSITION CONTROL: error dx = {dx}  |  error dy{dy}\n")

    rospy.loginfo(f"POSITION CONTROL: output velocidade linear = {cmd_vel.linear.x}  |  angular = {cmd_vel.angular.z}")

    cmd_vel.linear.x = ((1-abs(orientation_error)/math.pi)*(MAX_VEL - MIN_VEL) + MIN_VEL) * motion_direction
    cmd_vel.angular.z = angular_vel.output(KP_ANGULAR, KI_ANGULAR, KD_ANGULAR, orientation_error)

    # if (math.hypot(dx, dy) < 0.1): 
    #     cmd_vel.linear.x = 0
    #     cmd_vel.angular.z = 0    
    #     cmd_vel_pub.publish(cmd_vel)
    #     active_pid = False    

    if (active_pid):

        # print(f"VEL LINEAR = {cmd_vel.linear.x}") 
        # print(f"VEL ANGULAR = {cmd_vel.angular.z}")
        cmd_vel_pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
        rospy.init_node('position_controller', anonymous=True)
        rate = rospy.Rate(50)

        # rospy.Subscriber("/control/on",Bool,turn_on_controller_callback)

        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/goal_manager/goal/current", PoseStamped, setpoint_callback)
        rospy.Subscriber("/navigation/on",Bool, turn_on_pid_callback)
        
        while not rospy.is_shutdown():
            position_control()
            rate.sleep()

    
    except rospy.ROSInterruptException:
        pass
