from pid_control import PositionController

import rospy
import math
import tf

from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, Quaternion

# ----- constants pid controller -> linear 
KP_linear = 1 #0.25
KI_linear = 0.1 #0.01
KD_linear = 0.1 #0

# ----- constants pid controller -> angular 
KP_angular = 1.1#6.3 #0.5
KI_angular = 0.1#1
KD_angular = 0#0

# ----- setpoints / goal (x, y, theta)
goal_pose = Pose2D
goal_pose.x = 0
goal_pose.y = 0
goal_pose.theta = 0

# ----- current robot pose/orientation (x, y, theta)
current_pose = Pose2D
current_quaternion = Quaternion

# ----- tolerance
Tolerance_linear = 0.1          # in meters 
Tolerance_angular = math.pi/2     # in rad 
Distance_to_goal = 0.20            # in meters, for the exit point

# ----- machine states
active_pid = True

# ------ pid config 
angular = PositionController(KP_angular, KI_angular, KD_angular)  
linear = PositionController(KP_linear, KI_linear, KD_linear)    

# ------ publishers 
error_angular_pub = rospy.Publisher("/control/position/debug/angular/error", Float32, queue_size = 10)
error_linear_pub = rospy.Publisher("control/position/debug/linear/error", Float32, queue_size = 10)

# ------ subcribers 
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

# ------ messages 
error_angular_msg = Float32()
error_linear_msg = Float32()

vel_msg = Twist()

def turn_on_controller_callback(msg):
    global active_pid
    active_pid = msg.data

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

def setpoint_callback(msg): 
    global goal_pose
    goal_pose.x = msg.x
    goal_pose.y = msg.y 
    goal_pose.theta = msg.theta

def odom_callback(odom_msg): 
    global current_pose
    global current_quaternion 
    global goal_pose

    current_pose.x = odom_msg.pose.pose.position.x
    current_pose.y = odom_msg.pose.pose.position.y 
    current_quaternion = odom_msg.pose.pose.orientation

    # transforma quartenion em angulo de euler, e retira só o valor de yaw. 
    current_pose.theta = tf.transformations.euler_from_quaternion([current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w])[2]
    
    # distance to the goal 
    delta_x = goal_pose.x - current_pose.x
    delta_y = goal_pose.y - current_pose.y 

    error_linear = math.hypot(delta_x, delta_y)

    # theta angle to reach the current goal 
    theta_entrance = math.atan2(delta_x,delta_y) - current_pose.theta

    # ajust signal of the error_linear
    if (goal_pose.x < current_pose.x): 
        error_linear = - error_linear
    
    # if the robot is close enought to the goal, the error angular must consider de exit angle now
    if (error_linear < Distance_to_goal): 
        # theta angle for leave de currente goal and go to the next one
        theta_exit = goal_pose.theta - current_pose.theta
    else: 
        theta_exit = 0
    
    # calcula o ângulo total e ajusta para o intervalo [-pi, pi]
    error_angular = current_pose.theta + theta_entrance + theta_exit
    error_angular = math.atan2(math.sin(error_angular), math.cos(error_angular))

    # msg for the error topic 
    error_angular_msg.data = error_angular
    error_linear_msg.data = error_linear
    
    # publish the error
    error_angular_pub.publish(error_angular_msg)
    error_linear_pub.publish(error_linear_msg)

    # while the robot is out the tolerance, compute PID, otherwise, send 0 
    if (abs(error_linear) > Tolerance_linear and abs(error_angular) > Tolerance_angular):
        vel_msg.angular.z = angular.output(KP_angular, KI_angular, KD_angular, error_angular)
        vel_msg.angular.x = linear.output(KP_linear, KI_linear, KD_linear, error_linear)
    else: 
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0 
        print("no objetivo")

    # publish message if pid is active
    if (active_pid): 
        cmd_vel_pub.publish(vel_msg)

    # debug
    print(goal_pose.x)
    print(f"erro angular: {round(error_angular,3)} | error linear: {round(error_linear,3)} | vel_linear: {round(vel_msg.linear.x,3)} | vel_angular: {round(vel_msg.angular.z,0)}")
 

def controller_position():
    rospy.init_node('position_controller', anonymous=True)

    rospy.Subscriber("/control/on",Bool,turn_on_controller_callback)

    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/control/position/setup/goal", Pose2D, setpoint_callback)

    rospy.Subscriber("/control/position/setup/linear/kp", Float32, kp_linear_callback)
    rospy.Subscriber("/control/position/setup/linear/ki", Float32, ki_linear_callback)
    rospy.Subscriber("/control/position/setup/linear/kd", Float32, kd_linear_callback)

    rospy.Subscriber("/control/position/setup/angular/kp", Float32, kp_angular_callback)
    rospy.Subscriber("/control/position/setup/angular/ki", Float32, kd_angular_callback)
    rospy.Subscriber("/control/position/setup/angular/kd", Float32, ki_angular_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        controller_position()
    except rospy.ROSInterruptException:
        pass