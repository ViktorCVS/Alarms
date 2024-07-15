#!/usr/bin/env python3

import rospy

from gazebo_msgs.msg import LinkStates
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import random
import numpy as np
from math import sin,cos,pi
from tf.transformations import euler_from_quaternion

class CompareTrajectory():
    def __init__(self):

        rospy.init_node('ur5_compare_trajectory')

        # Criar um publisher para o tópico de comando de trajetória.
        self.pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        
        # Criar um subscriber para o tópico de posição das juntas.
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.get_link_callback)

        self.control = True

        # Esperar pelo publisher e subscriber se conectar
        rospy.sleep(1)

        self.inital_pose = [0.815829623,0.191638381,0.087551819]

        rospy.Timer(rospy.Duration(15), self.timer_callback)  


    def timer_callback(self,event): 

        traj = JointTrajectory()

        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        point = JointTrajectoryPoint()

        j1 = random.uniform(-pi,pi)
        j2 = random.uniform(0,-pi)

        if j2 > -pi/2:
            j3 = random.uniform(0,-pi/2)
        elif j2 < -pi/2:
            j3 = random.uniform(0,pi/2)

        j4 = random.uniform(0,-pi)

        j5 = random.uniform(-pi,pi)

        j6 = random.uniform(-pi,pi)

        point.positions = [j1, j2, j3, j4, j5, j6]

        point.time_from_start = rospy.Duration(2.5)

        traj.points.append(point)

        ##

        d1 = 0.089159
        a2 = 0.424994
        a3 = 0.392236
        d3 = 0.109051
        d4 = 0.0946455
        d5 = 0.0823  

        T1 = np.matrix([[cos(j1),-sin(j1),0,0],[sin(j1),cos(j1),0,0],[0,0,1,d1],[0,0,0,1]])
        T2 = np.matrix([[cos(j2),-sin(j2),0,0],[0,0,-1,0],[sin(j2),cos(j2),0,0],[0,0,0,1]])
        T3 = np.matrix([[cos(j3),-sin(j3),0,a2],[sin(j3),cos(j1),0,0],[0,0,1,0],[0,0,0,1]])
        T4 = np.matrix([[cos(j4),-sin(j4),0,a3],[sin(j4),cos(j4),0,0],[0,0,1,d3],[0,0,0,1]])
        T5 = np.matrix([[cos(j5),-sin(j5),0,0],[0,0,-1,-d4],[sin(j5),cos(j5),0,0],[0,0,0,1]])
        T6 = np.matrix([[cos(j6),-sin(j6),0,0],[0,0,1,d5],[-sin(j5),-cos(j5),0,0],[0,0,0,1]])

        T = T1*T2*T3*T4*T5*T6

        ##

        new_position = T * self.position.T
        new_position = [new_position[0]/new_position[3],new_position[1]/new_position[3],new_position[1]/new_position[3]]
        print("pos[k+1]: ",new_position)
        print("pos[k]: ",self.position)
        print()

        self.pub.publish(traj)


    def get_link_callback(self,msg):

        position = msg.pose[-1].position
        self.position = np.matrix([position.x,position.y,position.z,1])
        
        orientation = msg.pose[-1].orientation
        orientation = [orientation.x,orientation.y,orientation.z,orientation.w]
        self.orientation = list(euler_from_quaternion(orientation))
 

if __name__ == '__main__':
    try:
        init_compare = CompareTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
