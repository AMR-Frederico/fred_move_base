

import rospy
import math
import tf

from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, Quaternion

from time import time

class PIDController: 
    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KD = KD 
        self.KI = KI

        self.time = time()
        self.last_time = time()
        self.delta_time = 0

        self.error = 0
        self.last_error = 0
        self.delta_error = 0

        self.integral = 0
    
    def proporcional(self):

        return self.KP * self.error
    
    def integrative(self): 

        self.integral += self.error * self.delta_time
            #anti wind-up
        if(self.integral > 1.5 or self.integral < -1.5):
            self.integral = 0
        # print(self.integral)
        return self.integral * self.KI

    def derivative(self):
        self.delta_error = self.error - self.last_error

        if(self.delta_error != 0):
            self.delta_error = self.delta_error/self.delta_time
        else:
            self.delta_error = 0
        return self.delta_error*self.KD
            
    def output(self, kp, ki, kd, error):
        self.KP = kp
        self.KI = ki
        self.KD = kd

        self.error = error

        self.time = time()
        self.delta_time = self.time - self.last_time

        if (self.error != 0):
            output = self.proporcional() + self.integrative() + self.derivative()
        else: 
            output = self.proporcional() + self.derivative()
        
        self.last_error = self.error
        self.last_time = self.time

        return output

# -------------------------------------------------- fim da classe

# ----- constantes PID angular
KP_angular = 0.5#6.3 #0.5
KI_angular = 0#1
KD_angular = 0#0

# ----- constants pid controller -> linear 
KP_linear = 0.1 #0.25
KI_linear = 0#0.01
KD_linear = 0 #0

# limites de vel linear
max_linear = 10
min_linear = 0

# ----- setpoints / goal (x, y, theta)
goal_pose = Pose2D()
goal_pose.x = 0
goal_pose.y = 0
goal_pose.theta = 0
goal_quaternion = Quaternion()

# ----- current robot pose/orientation (x, y, theta)
current_pose = Pose2D()
current_quaternion = Quaternion()

# ----- tolerance
Tolerance_linear = 0.5         # in meters 
Tolerance_angular = math.pi/2        # in rad 
Distance_to_goal = 0           # in meters, for the exit point

active_pid = True

# ------ pid config 
angular = PIDController(KP_angular, KI_angular, KD_angular)   
linear = PIDController(KP_linear, KI_linear, KD_linear)

# ------ publishers 
error_angular_pub = rospy.Publisher("/control/position/debug/angular/error", Float32, queue_size = 10)
error_linear_pub = rospy.Publisher("control/position/debug/linear/error", Float32, queue_size = 10)
# goal_reached_pub = rospy.Publisher("/goal_manager/goal/reached", Bool, queue_size = 10)

# ------ messages 
error_orientation_msg = Float32()
error_linear_msg = Float32()
goal_reached_msg = Bool()
vel_msg = Twist()

# ------ subcribers 
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


# reduzir ângulo entre -pi e pi
def reduce_angle(angle):
    # angle = angle % 2*math.pi
    if angle > math.pi:
        angle = angle - 2*math.pi
    if angle < -math.pi:
        angle = angle + 2*math.pi
    return angle

# ativa o controle de posição
def turn_on_controller_callback(msg):
    global active_pid
    active_pid = msg.data

# troca das constantes do pid angular de forma dinamica
def kp_angular_callback(msg):
    global KP_angular 
    KP_angular = msg.data

# troca das constantes do pid angular de forma dinamica
def kd_angular_callback(msg):
    global KI_angular 
    KI_angular = msg.data 

# troca das constantes do pid angular de forma dinamica
def ki_angular_callback(msg):
    global KD_angular 
    KD_angular = msg.data

# atualiza o objetivo
def setpoint_callback(msg): 
    global goal_pose
    goal_pose.x = msg.pose.position.x
    goal_pose.y = msg.pose.position.y 

    goal_quaternion = msg.pose.orientation 
    goal_pose.theta = tf.transformations.euler_from_quaternion([goal_quaternion.x, goal_quaternion.y, goal_quaternion.z, goal_quaternion.w])[2]

# posição atual do robo
def odom_callback(odom_msg): 
    global current_pose
    global current_quaternion 

    # posição atual do robo de acordo com a odometria
    current_pose.x = odom_msg.pose.pose.position.x
    current_pose.y = odom_msg.pose.pose.position.y 
    current_quaternion = odom_msg.pose.pose.orientation
    
    # transforma quaternion em ângulo de euler, e retira só o valor de yaw. 
    current_pose.theta = tf.transformations.euler_from_quaternion([current_quaternion.x, current_quaternion.y, current_quaternion.z, current_quaternion.w])[2]

def position_control(): 

    global goal_pose
    global current_pose

    # distancia até o objetivo, em relação ao eixo x e ao eixo y
    dx = goal_pose.x - current_pose.x
    dy = goal_pose.y - current_pose.y

    # calculo do modulo do erro linear
    error_linear = math.hypot(dx, dy)
    
    angulo_erro = math.atan2(dy,dx)
    angulo_robo = current_pose.theta

    # proj_x = dx * math.cos(angulo_erro) + dy * math.sin(angulo_erro)  # se proj_x > 0, vetor esta no sentido positivo de x
    # proj_y = -dx * math.sin(angulo_erro) + dy * math.cos(angulo_erro) # se proj_y > 0, vetor esta no sentido positivo de y


   # while the robot is out the tolerance, compute PID, otherwise, send 0
    error_orientation = reduce_angle(angulo_erro - angulo_robo)
    
    # if abs(error_orientation)>math.pi/3:   #força o robo a rotacionar ate que esteja alinhado ao angulo
    #     vel_msg.linear.x = 0
    #     if error_orientation > 0:
    #         vel_msg.angular.z = 1
    #     else:
    #         vel_msg.angular.z = -1
    
    if (abs(error_linear) > Tolerance_linear):

        # mapea a velocidade linear em função do erro de orientação, 
        # se o erro for máximo -> vel_linear mínima
        # sem o erro for mínimo -> vel_linear máxima
        vel_msg.linear.x = (1-abs(error_orientation)/math.pi)*(max_linear - min_linear) + min_linear
        
        # vel_msg.linear.x = linear.output(KP_linear, KI_linear, KD_linear, error_linear) 

        # com isso só a velocidade angular passa pelo
        vel_msg.angular.z = angular.output(KP_angular, KI_angular, KD_angular, error_orientation)
        # goal_reached_msg.data = False
    else: 
        # goal_reached_msg.data = True
        print("no objetivo")
    
    # goal_reached_pub.publish(goal_reached_msg)

    # publish message if pid is active
    if (active_pid): 
        cmd_vel_pub.publish(vel_msg)

    # publish the error
    error_orientation_msg.data = error_orientation
    error_linear_msg.data = error_linear
    error_angular_pub.publish(error_orientation_msg)
    error_linear_pub.publish(error_linear_msg)

    # debug
    # print("CONTROL POSITION NODE -------------------------------------------------")
    # print(f"SETPOINT -> x:{goal_pose.x} y:{goal_pose.y} theta:{round(goal_pose.theta,2)}")
    # print(f"CURRENT POSITION -> x:{round(current_pose.x,2)} y:{round(current_pose.y)} theta:{round(angulo_robo,3)}")
    # print(f"DELTA -> x:{round(dx,2)}  y:{round(dy,2)} angulo do erro:{round(angulo_erro,3)} ")
    # print(f"ERRO ORIENTACAO -> {error_orientation}")
    # print(f"THETA -> dth:{round(angulo_erro,2)}  th:{round(math.atan2(dx,dy),2)}")
    # print(f"ERROR -> linear:{round(error_linear,2)}")
    # print(f"VELOCITY OUTPUT -> linear:{round(vel_msg.linear.x,2)}  angular:{round(vel_msg.angular.z,2)}")
    # # print(f"PROJECTION -> x: {round(proj_x,2)} y:{round(proj_y,2)}  ")
    # print("\n")

# def controller_position():




if __name__ == '__main__':
    try:
        rospy.init_node('position_controller', anonymous=True)
        rate = rospy.Rate(100)
        rospy.Subscriber("/control/on",Bool,turn_on_controller_callback)

        rospy.Subscriber("/odom", Odometry, odom_callback)
        rospy.Subscriber("/goal_manager/goal/current", PoseStamped, setpoint_callback)

        rospy.Subscriber("/control/position/setup/angular/kp", Float32, kp_angular_callback)
        rospy.Subscriber("/control/position/setup/angular/ki", Float32, kd_angular_callback)
        rospy.Subscriber("/control/position/setup/angular/kd", Float32, ki_angular_callback)
        
        while not rospy.is_shutdown():
            position_control()
            rate.sleep()

    
    except rospy.ROSInterruptException:
        pass