#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from time import sleep

def talker_init_main(x, y, theta):
    
    # escrevo mensagem no formato PoseWithCovarianceStamped no topico initialpose
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    rate = rospy.Rate(10)

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    quat = quaternion_from_euler(0, 0, theta)
    msg.pose.pose.orientation.x = quat[0]
    msg.pose.pose.orientation.y = quat[1]
    msg.pose.pose.orientation.z = quat[2]
    msg.pose.pose.orientation.w = quat[3]
    
    if not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()
        rospy.loginfo(msg) #A posição inicial só é aceita após a segunda publicação (?)
        pub.publish(msg)
    
def set_goal():

    # defino os destinos com as posicoes x e y e orientacao theta
    x = [-3.5, -4.0, -3.0]
    y = [-0.5, -1.0, 1.0]
    theta = [3.14, 1.57, 0.0]

    goals = []

    for i in range(3):
        goals.append(PoseStamped())
        goals[i].header.frame_id = 'map'
        goals[i].pose.position.x = x[i]
        goals[i].pose.position.y = y[i]
        quat = quaternion_from_euler(0, 0, theta[i])
        goals[i].pose.orientation.x = quat[0]
        goals[i].pose.orientation.y = quat[1]
        goals[i].pose.orientation.z = quat[2]
        goals[i].pose.orientation.w = quat[3]
    return goals


def listener_callback(data):
    global status
    status = data.status.status

def talker_route_main(goal):
    
    # escrevo mensagem de estrutura PoseStamped no topico move_base_simple/goal
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(4)
    
    
    if not rospy.is_shutdown():
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()
        rospy.loginfo(goal) #O destino só é aceito após a segunda publicação
        pub.publish(goal)
          
if __name__ == '__main__':
        
    initial_x = -3.0
    initial_y = 1.0
    initial_theta = 0.0

    rospy.init_node('plant_goal')
    
    talker_init_main(initial_x, initial_y, initial_theta)

    status = 0
    poses = set_goal()
    
    for i in range(3):
        talker_route_main(poses[i])
        sleep(2)
        while not rospy.is_shutdown():
            rospy.Subscriber('/move_base/result', MoveBaseActionResult, listener_callback)
            if status == 3:
                break 
