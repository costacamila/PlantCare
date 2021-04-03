#!/usr/bin/env python3                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        #!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
from time import sleep

def set_goal():

    # defino os destinos com as posicoes x e y e orientacao theta
    x = [5.2, -2.3]
    y = [-0.3, 3.0]
    theta = [0.0, 1.57]

    goals = []

    for i in range(2):
        goals.append(PoseStamped())
        goals[i].header.frame_id = 'housemap'
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


def talker_main(goal):
    
    # escrevo mensagem de estrutura PoseStamped no topico move_base_simple/goal
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(4)
    
    
    if not rospy.is_shutdown():
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()
        rospy.loginfo(goal) #O destino só é aceito após a segunda publicação (?)
        pub.publish(goal)
    
          
if __name__ == '__main__':
        
    status = 0
    # crio as poses                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
    poses = set_goal()
    # inicio o no uma unica vez
    rospy.init_node('pose_goal')
    
    for i in range(2):
        talker_main(poses[i])
        sleep(2)
        while not rospy.is_shutdown():
            rospy.Subscriber('/move_base/result', MoveBaseActionResult, listener_callback)
            if status == 3:
                break 
    