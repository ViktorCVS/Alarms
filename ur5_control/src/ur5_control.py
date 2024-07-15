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

        rospy.Timer(rospy.Duration(5), self.timer_callback)  


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
        point.positions = [0, 0, 0, 0, 0, 0]

        point.time_from_start = rospy.Duration(2.5)

        traj.points.append(point)

        ##

        d1 = 0.08915895487108214+0.1000000368892521
        a2  = 0.424994
        dz2 = -0.002161932
        a3 = 0.392236
        d3 = 0.109051
        dz = -0.004635843 
        dx = -0.001118244
        d4 = -0.09464348
        d5 = 0.0823  

        j1,j2,j3,j4,j5,j6 = pi/2,0,0,0,0,0


        T1 = np.matrix([[cos(j1),-sin(j1),0       ,0         ],
                        [sin(j1),cos(j1) ,0       ,0         ],
                        [0      ,0       ,1       ,d1        ],
                        [0      ,0       ,0       ,1         ]])


        T2 = np.matrix([[cos(j2),0       ,sin(j2) ,0         ],
                        [sin(j2),0       ,-cos(j2),0         ],
                        [0      ,1       ,0       ,0         ],
                        [0      ,0       ,0       ,1         ]])


        T3 = np.matrix([[cos(j3),-sin(j3),0       ,a2*cos(j3)],
                        [sin(j3),cos(j3) ,0       ,a2*sin(j3)],
                        [0      ,0       ,1       ,0         ],
                        [0      ,0       ,0       ,1         ]])

        T4 = np.matrix([[cos(j4),-sin(j4),0       ,a3*cos(j4)],
                        [sin(j4),cos(j4) ,0       ,a3*sin(j4)],
                        [0      ,0       ,1       ,d3        ],
                        [0      ,0       ,0       ,1         ]])

        T5 = np.matrix([[cos(j5),0       ,sin(j5) ,0         ],
                        [sin(j5),0       ,-cos(j5),0         ],
                        [0      ,1       ,0       ,d4        ],
                        [0      ,0       ,0       ,1         ]])

        T6 = np.matrix([[cos(j6),0       ,-sin(j6),0         ],
                        [sin(j6),0       ,cos(j6) ,0        ],
                        [0      ,-1      ,0       ,d5         ],
                        [0      ,0       ,0       ,1         ]])



        T = T1@T2@T3@T4@T5@T6

        print(T_end)
        print(T)

        ##

        new_position = T * self.position.T
        new_position = T * np.matrix([0,0,0,1]).T
        print(new_position)
        new_position = [new_position[0]/new_position[3],new_position[1]/new_position[3],new_position[1]/new_position[3]]
        #print("pos[k+1]: ",new_position)
        print("pos[k]: ",self.position)
        print()

        self.pub.publish(traj)


    def get_link_callback(self,msg):

        position = msg.pose[-1].position
        self.position = np.matrix([position.x,position.y,position.z,1])
        
        orientation = msg.pose[-1].orientation
        orientation = [orientation.x,orientation.y,orientation.z,orientation.w]
        self.orientation = list(euler_from_quaternion(orientation))

    def Tx(self,dx):
        return np.matrix([[1,0,0,dx],
                          [0,1,0,0],
                          [0,0,1,0],
                          [0,0,0,1]])
    
    def Ty(self,dy):
        return np.matrix([[1,0,0,0],
                          [0,1,0,dy],
                          [0,0,1,0],
                          [0,0,0,1]])
    
    def Tz(self,dz):
        return np.matrix([[1,0,0,0],
                          [0,1,0,0],
                          [0,0,1,dz],
                          [0,0,0,1]])
    
    def Rx(self,theta):
        return np.matrix([[1,0,0,0],
                          [0,cos(theta),-sin(theta),0],
                          [0,sin(theta),cos(theta),0],
                          [0,0,0,1]])
    
    def Ry(self,theta):
        return np.matrix([[cos(theta),0,sin(theta),0],
                          [0,1,0,0],
                          [-sin(theta),0,cos(theta),0],
                          [0,0,0,1]])
    
    def Rz(self,theta):
        return np.matrix([[cos(theta),-sin(theta),0,0],
                          [sin(theta),cos(theta),0,0],
                          [0,0,1,0],
                          [0,0,0,1]])
 

if __name__ == '__main__':
    try:
        init_compare = CompareTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
