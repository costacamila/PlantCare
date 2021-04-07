#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from time import sleep

def talker_init_main(x, y, theta):
    
    # publisher que escreve mensagem de estrutura PoseWithCovarianceStamped no tópico initialpose
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
        # a posição inicial só é aceita após a segunda publicação
        rospy.loginfo(msg)
        pub.publish(msg)
    
def set_goal(x, y, theta, plants):

    goals = []

    for i in range(plants+1):
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
    # variavél global pois a listener_callback está sendo chamada indiretamente sendo impossível retornar valor
    global status
    status = data.status.status

def talker_route_main(goal):
    
    # publisher que escreve mensagem de estrutura PoseStamped no tópico move_base_simple/goal
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rate = rospy.Rate(4)
    
    
    if not rospy.is_shutdown():
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()
        # o destino só é aceito após a segunda publicação
        rospy.loginfo(goal)
        pub.publish(goal)
          
if __name__ == '__main__':

    initial = [-2.0, -0.5, 0.0]

    x = []
    y = []
    theta = []

    plants = eval(input("Digite a quantidade de plantas a serem regadas: "))

    for i in range(plants):
        x.append(eval(input("Digite a posição no eixo x para o robô navegar até a planta "+str(i+1)+": ")))
        y.append(eval(input("Digite a posição no eixo y para o robô navegar até a planta "+str(i+1)+": ")))
        theta.append(eval(input("Digite a rotação em radianos para o robô se orientar em relação a planta "+str(i+1)+": ")))

    x.append(initial[0])
    y.append(initial[1])
    theta.append(initial[2])

    # opção hard coded (comentar da linha 76 até 89 para funcionar)
    # plants = 3
    # x = [-0.5, 0.5, 1.5]
    # y = [1.0, 0.0, -1.0]
    # theta = [3.14, 3.14, 3.14]

    # inicio o nó plant_routine
    rospy.init_node('plant_routine')
    
    talker_init_main(initial[0], initial[1], initial[2])

    status = 0
    poses = set_goal(x, y, theta, plants)
    
    for i in range(plants+1):
        talker_route_main(poses[i])
        sleep(2)
        while not rospy.is_shutdown():
            # enquanto o nó estiver ativo se inscreve no tópico move_base/result com mensagem de estrutura MoveBaseActionResult
            # chama a listener_callback passando a mensagem de estrutura MoveBaseActionResult
            rospy.Subscriber('/move_base/result', MoveBaseActionResult, listener_callback)
            if status == 3:
                break
