#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool
seq = 0  # Variável global para controlar o valor de sequência
path = Path()

 # Criar um publisher para o tópico 'path'
path_pub = rospy.Publisher('/path', Path, queue_size=10)


def reset_goals_callback(msg):
        global seq
        reset = msg.data 
        if(reset):
             path.poses.clear()
             seq = 0


def odometry_callback(msg):
    global seq, path


    p = PoseStamped()
    p.pose.position.x = msg.pose.pose.position.x
    p.pose.position.y = msg.pose.pose.position.y
    p.pose.position.z = msg.pose.pose.position.z

    p.pose.orientation.x = msg.pose.pose.orientation.x
    p.pose.orientation.y = msg.pose.pose.orientation.y
    p.pose.orientation.z = msg.pose.pose.orientation.z
    p.pose.orientation.w = msg.pose.pose.orientation.w

    p.header.stamp = rospy.Time.now()
    p.header.seq = seq
    seq = seq+1

    path.poses.append(p)
    path.header.stamp = rospy.Time.now()
    path.header.frame_id = "odom"
    path_pub.publish(path)
    

    # Publicar a mensagem Path no tópico 'path'



if __name__ == '__main__':
    rospy.init_node('odometry_to_path_node')

    # Subscrever ao tópico de odometria
    rospy.Subscriber('odom', Odometry, odometry_callback)
    rospy.Subscriber("/goal_manager/goal/reset", Bool, reset_goals_callback)
    rospy.spin()
    
