import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

desired_position = 0.0
tolerance = 0.2

def position_callback(position_msg):
    global desired_position
    desired_position = position_msg.data

def odom_callback(odom_msg):
    global desired_position, tolerance
    current_position = odom_msg.pose.pose.position.x
    error = desired_position - current_position
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    if abs(error) < tolerance:
        vel_msg.linear.x = 0.0
    else:
        Kp = 0.5
        vel_msg.linear.x = Kp * error
    pub.publish(vel_msg)

def control_position():
    rospy.init_node('position_controller', anonymous=True)
    rospy.Subscriber("/control/position/x", Float64, position_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        control_position()
    except rospy.ROSInterruptException:
        pass




# ######################################################################################################################################################33

# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# # from sensor_msgs import range
# # from sensor_msgs.msg import Joy
# from std_msgs.msg import Float32
# from std_msgs.msg import Int32

# error_msg = Float32

# cmd_vel_msg = Twist()
# current_x = 0
# setPoint_x = 0
# input_x = 0 
# last_input_x = 0

# kP= 0.2
# tolerancia = 0.2 #[m]

# def proporcional(error):
#     global kP
#     proporcional = error*kP
#     # debug_control_p_sub.publish(proporcional)

#     return proporcional

# def control(input_x, current_x):
#     error = input_x - current_x

#     # error_msg.data = error
#     # debug_control_error_sub.publish(error_msg)

#     output = proporcional(error)
#     # debug_control_out_sub.publish(output)



#     if(abs(error) < tolerancia ):
#         output = 0
#         print(f"STOP| X:{current_x}| Setpoint: {input_x} | error : {error}| out: {output} ")
#     else:
#         print(f"MOVE| X:{current_x}| Setpoint: {input_x} | error : {error}| out: {output} ")
        

#     cmd_vel_msg.linear.x = output 
#     cmd_vel_pub.publish(cmd_vel_msg)

# def update_x(msg):
#     global last_input_x
#     global setPoint_x

#     input_x = msg.data 
#     if(input_x!=last_input_x):
#         print("CHANGE VALUE")
#         setPoint_x = input_x
#     last_input_x = input_x

# def get_position(msg):
#     global setPoint_x
#     current_x = msg.pose.pose.position.x
#     control(setPoint_x,current_x)
    
    

# def publish_vel(x):
#     cmd_vel_msg.linear.x = x 
#     cmd_vel_pub.publish(cmd_vel_msg)


# #publish twist 

# rospy.init_node('Position_control_node')
# cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
# debug_control_error_sub = rospy.Publisher('control/debug/error', Float32, queue_size=10)
# debug_control_p_sub = rospy.Publisher('control/debug/p', Float32, queue_size=10)
# debug_control_out_sub = rospy.Publisher('control/debug/out', Float32, queue_size=10)

# r = rospy.Rate(10)

# if __name__ == '__main__':
#      while not rospy.is_shutdown():
#         #get new x position 
#         input_x_sub = rospy.Subscriber(
#             "control/position/x", Float32, update_x)
#         #get current position
#         odom = rospy.Subscriber("odom",Odometry, get_position)

#         r.sleep()