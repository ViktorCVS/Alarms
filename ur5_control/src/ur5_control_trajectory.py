#!/usr/bin/env python3

import rospy                                                                 # rospy para utilizar funções do ROS.

from gazebo_msgs.msg import LinkStates                                       # LinkStates para ler o Ground Truth do Gazebo para cada junta, incluindo o executor.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint        # JointTrajectory para enviar o comando de trajetória e JointTrajectoryPoint para gerar o ponto da trajetória.

import random                                                                # random para escolher aleatoriamente um ângulo para cada junta.
import numpy as np                                                           # numpy para trabalho matricial.
from collections import deque                                                # deque para criar uma fila de amostras de velocidade.
from math import sin,cos,pi, ceil, inf, acos, asin, atan2, sqrt              # math para uso das funções seno, cosseno e teto, além do valor de pi e infinito.
from tf.transformations import euler_from_quaternion                         # euler_from_quaternion para converter a orientação de quaternion para euler.
from scipy.spatial.transform import Rotation as R                            # R para trabalhar com rotações espaciais.


class CompareTrajectory():
    def __init__(self):

        # Nomeando o nó.
        rospy.init_node('ur5_compare_trajectory') 

        # Intervalo de tempo para enviar a referência de trajetória.
        tempo_referencia = 6

        # Tempo para que a trajetória seja concluída quando a referência for enviada.
        self.tempo_trajetoria = 3

        # Contador de iteração.
        self.contador = 0
        self.inner_contador = 0

        # declaração das variáveis das juntas
        self.j1 = 0
        self.j2 = 0
        self.j3 = 0
        self.j4 = 0
        self.j5 = 0
        self.j6 = 0

        # declaração das variáveis de posição, velocidade e orientação
        self.velocity_samples = []
        self.pos = []
        self.vel = []
        self.acc=[]

        # flags para controle de execução dos dados
        self.flag_dados = False
        self.block = True

        # Criar um publisher para o tópico de comando de trajetória.
        self.pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        
        # Criar um subscriber para o tópico de posição das juntas.
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.get_link_callback)


        # Esperar pelo publisher e subscriber se conectarem.
        rospy.sleep(0.5)

        # Timer para enviar a referência de posição periodicamente.
        rospy.Timer(rospy.Duration(tempo_referencia), self.timer_callback)

    def quintic_polynomial(self, t0, tf, theta0, thetaf, v0, vf, a0, af):

        # Solução do polinômio de quinta ordem para o planejamento de trajetória.

        T = tf - t0
        A = np.array([
            [1, t0, t0**2, t0**3, t0**4, t0**5],
            [0, 1, 2*t0, 3*t0**2, 4*t0**3, 5*t0**4],
            [0, 0, 2, 6*t0, 12*t0**2, 20*t0**3],
            [1, tf, tf**2, tf**3, tf**4, tf**5],
            [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
            [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]
        ])
        B = np.array([theta0, v0, a0, thetaf, vf, af])
        return np.linalg.solve(A, B)

    def generate_trajectory(self, q0, qf, v0, vf, a0, af, duration):

        # Geração de trajetória para o planejamento de movimento.

        coeffs = []
        for i in range(6):  # Assuming a 6-DOF robot like UR5
            coeffs.append(self.quintic_polynomial(0, duration, q0[i], qf[i], v0[i], vf[i], a0[i], af[i]))

        # Sample the trajectory
        traj = JointTrajectory()
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        
        num_samples = 100
        times = np.linspace(0, duration, num_samples)
        for t in times:
            point = JointTrajectoryPoint()
            positions = []
            for i in range(6):
                position = np.polyval(coeffs[i][::-1], t)  # Calculate position at time t
                positions.append(position)
            point.positions = positions
            point.time_from_start = rospy.Duration(t)
            traj.points.append(point)
        
        return traj  


    def timer_callback(self,event): 

        # ----- Definindo as medidas estruturais do robô UR5, considerando ajustes de simulação e o sistema de coordenadas. -----

        d1 = 0.08915895487108214+0.1000000368892521

        a2 = 0.424994
        d2 = 0.002161932

        a3 = 0.392236
        d3 = 0.109051

        de = 0.004635303

        d4 = 0.09464348

        ax = 0.001354042

        d5 = 0.0823 

        # ----------------------------------------------------------------------------------------------------------------------- 

        # Fim do movimento - salvando os dados
        if self.contador%5 == 4:
            self.flag_dados = True

        # Início do movimento, escolhendo a posição inicial e final das juntas
        if self.contador%4 == 0 or self.contador%4 == 1:

            self.j1 = random.uniform(-pi,pi)
            self.j2 = random.uniform(0,-pi)

            if self.j2 > -pi/2:
                self.j3 = random.uniform(0,-pi/2)
            elif self.j2 < -pi/2:
                self.j3 = random.uniform(0,pi/2)

            self.j4 = random.uniform(0,-pi)

            self.j5 = random.uniform(0,2*pi)

            self.j6 = random.uniform(-pi,pi)

        # alocando o valor aleatório na posição inicial ou final da junta e recuperando a pose do executor
        if self.contador%4 == 0:
            self.q0 = [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]  # Initial joint configuration
        elif self.contador%4 == 1:
            self.pos_zero = self.position
            self.orien_zero = self.orientation
            self.qf = [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]
        elif self.contador%4 == 2:
            self.pos_final = self.position
            self.orien_final = self.orientation

        if self.contador%4 == 1 or self.contador%4 == 2:

            # Cinemática inversa para recuperar o valor das juntas a partir das posições dadas.

            if self.contador%4 == 1:
                self.posi = self.pos_zero
                self.orien = self.orien_zero
                self.q = self.q0
            elif self.contador%4 == 2:
                self.posi = self.pos_final
                self.orien = self.orien_final
                self.q = self.qf

            T = np.matrix([[cos(self.orien[2])*cos(self.orien[1]), cos(self.orien[2])*sin(self.orien[1])*sin(self.orien[0])-sin(self.orien[2])*cos(self.orien[0]), cos(self.orien[2])*sin(self.orien[1])*cos(self.orien[0])+sin(self.orien[2])*sin(self.orien[0]), self.posi[0]],
                        [sin(self.orientation[2])*cos(self.orien[1]), sin(self.orien[2])*sin(self.orien[1])*sin(self.orien[0])+cos(self.orien[2])*cos(self.orien[0]), sin(self.orien[2])*sin(self.orien[1])*cos(self.orien[0])-cos(self.orien[2])*sin(self.orien[0]), self.posi[1]],
                        [-sin(self.orien[1]), cos(self.orien[1])*sin(self.orien[0]), cos(self.orien[1])*cos(self.orien[0]), self.posi[2]],
                        [0, 0, 0, 1]])

            T_5 = T*np.matrix([[0],[0],[-d5],[1]])

            P_5 = [T_5[0,0],T_5[1,0],T_5[2,0]]

            self.tetha_1 = atan2(P_5[1],P_5[0])+acos(d3/sqrt(P_5[0]**2+P_5[1]**2))+pi/2
            tetha_1x = atan2(P_5[1],P_5[0])-acos(d3/sqrt(P_5[0]**2+P_5[1]**2))+pi/2

            if self.tetha_1 > 0: 
                self.tetha_1 -= pi 
            elif self.tetha_1 < 0: 
                self.tetha_1 += pi

            if tetha_1x > 0: 
                tetha_1x -= pi 
            elif tetha_1x < 0: 
                tetha_1x += pi

            if abs(self.q[0]-self.tetha_1)>=abs(self.q[0]-tetha_1x):
                self.tetha_1 = tetha_1x

            if abs(self.q[0]-self.tetha_1)>=abs(self.q[0]+self.tetha_1):
                self.tetha_1 *= -1

            rex = (-self.posi[0]*sin(self.tetha_1)+self.posi[1]*cos(self.tetha_1)-d3)/d5
            self.tetha_5 = acos(rex)

            if abs(self.tetha_5 - self.q[4]) >= abs(-self.tetha_5+2*pi -self.q[4]):
                self.tetha_5 = -self.tetha_5+2*pi

            if abs(self.q[4]-self.tetha_5)>=abs(self.q[4]+self.tetha_5):
                self.tetha_5 *= -1

            if self.orien[2] >= 0: alfa = self.orien[2]-pi
            elif self.orien[2] < 0: alfa = self.orien[2]+pi

            T = np.matrix([[cos(alfa)*cos(self.orien[1]), cos(alfa)*sin(self.orien[1])*sin(self.orien[0])-sin(alfa)*cos(self.orien[0]), cos(alfa)*sin(self.orien[1])*cos(self.orien[0])+sin(alfa)*sin(self.orien[0]), -self.posi[0]],
                        [sin(alfa)*cos(self.orien[1]), sin(alfa)*sin(self.orien[1])*sin(self.orien[0])+cos(alfa)*cos(self.orien[0]), sin(alfa)*sin(self.orien[1])*cos(self.orien[0])-cos(alfa)*sin(self.orien[0]), -self.posi[1]],
                        [-sin(self.orien[1]), cos(self.orien[1])*sin(self.orien[0]), cos(self.orien[1])*cos(self.orien[0]), self.posi[2]],
                        [0, 0, 0, 1]])
            T_i = np.linalg.inv(T)

            self.tetha_6 = atan2((-T_i[1,0]*sin(self.tetha_1)+T_i[1,1]*cos(self.tetha_1))/sin(self.tetha_5),(T_i[0,0]*sin(self.tetha_1)-T_i[0,1]*cos(self.tetha_1))/sin(self.tetha_5))

            if abs(self.q[5]-self.tetha_6)>=abs(self.q[5]+self.tetha_6):
                self.tetha_6 *= -1

            T56 = np.matrix([[cos(self.tetha_6),-sin(self.tetha_6),0,0],
                            [0,0,1,d5],
                            [-sin(self.tetha_6),-cos(self.tetha_6),0,0],
                            [0,0,0,1]])

            T45 = np.matrix([[cos(self.tetha_5),-sin(self.tetha_5),0,0],
                            [0,0,-1,-(d4+de)],
                            [sin(self.tetha_5),cos(self.tetha_5),0,0],
                            [0,0,0,1]])

            T01 = np.matrix([[cos(self.tetha_1),-sin(self.tetha_1),0,0],
                            [sin(self.tetha_1),cos(self.tetha_1),0,0],
                            [0,0,1,d1],
                            [0,0,0,1]])
            

            T14 = np.linalg.inv(T01)*T*np.linalg.inv(T56)*np.linalg.inv(T45)

            temp = (-T14[0,3]**2-T14[2,3]**2+a2**2+(a3-ax)**2)/(2*a2*(a3-ax))

            if temp > 1: temp = -((-1+temp)-1)
            elif temp < -1: temp = -((1+temp)+1)

            self.tetha_3 = acos(temp)
            tetha_3x = -acos(temp)

            if self.tetha_3 > 0: 
                self.tetha_3 -= pi 
            elif self.tetha_3 < 0: 
                self.tetha_3 += pi

            if tetha_3x > 0: 
                tetha_3x -= pi 
            elif tetha_3x < 0: 
                tetha_3x += pi

            if abs(self.q[2]-self.tetha_3)>=abs(self.q[2]-tetha_3x):
                self.tetha_3 = tetha_3x

            if abs(self.q[2]-self.tetha_3)>=abs(self.q[2]+self.tetha_3):
                self.tetha_3 *= -1

            self.tetha_2 = atan2(-T14[2,3],-T14[0,3])-asin((a3-ax)*sin(self.tetha_3)/sqrt(T14[0,3]**2+T14[2,3]**2))

            if abs(self.q[1]-self.tetha_2)>=abs(self.q[1]+self.tetha_2):
                self.tetha_2 *= -1

            T12 = np.matrix([[cos(self.tetha_2),-sin(self.tetha_2),0,0],
                            [0,0,-1,0],
                            [sin(self.tetha_2),cos(self.tetha_2),0,0],
                            [0,0,0,1]])

            T23 = np.matrix([[cos(self.tetha_3),-sin(self.tetha_3),0,a2],
                            [sin(self.tetha_3),cos(self.tetha_3),0,0],
                            [0,0,1,0],
                            [0,0,0,1]])

            T34 = np.linalg.inv(T23)*np.linalg.inv(T12)*T14

            self.tetha_4 = atan2(T34[1,0],T34[0,0])

            if abs(self.q[3]-self.tetha_4)>=abs(self.q[3]+self.tetha_4):
                self.tetha_4 *= -1


            # alocando a posição das juntas iniciais e finais pela cinemática inversa
            if self.contador%4 == 1:
                self.qi_i = [self.tetha_1, self.tetha_2, self.tetha_3, self.tetha_4, self.tetha_5, self.tetha_6]
            else:
                self.qi_f = [self.tetha_1, self.tetha_2, self.tetha_3, self.tetha_4, self.tetha_5, self.tetha_6]

        # Definindo as velocidades e acelerações iniciais e finais nulas
        v0 = [0, 0, 0, 0, 0, 0]
        vf = [0, 0, 0, 0, 0, 0] 
        a0 = [0, 0, 0, 0, 0, 0]
        af = [0, 0, 0, 0, 0, 0]

        if self.contador%4 == 0 or self.contador%4 == 1 or self.contador%4 == 2:
            
            # Enviando a referência de trajetória para o executor pelo algoritmo de planejamento de trajetória em ROS.

            traj = JointTrajectory()
            traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            point = JointTrajectoryPoint()

            if self.contador%4 == 0: point.positions = self.q0
            if self.contador%4 == 1: point.positions = self.qf
            if self.contador%4 == 2: point.positions = self.qi_i

            point.time_from_start = rospy.Duration(self.tempo_trajetoria)
            traj.points.append(point)
            self.pub.publish(traj)

        else:

            # Enviando a trajetória planejada para o executor pela trajetória quíntica

            traj = self.generate_trajectory(self.qi_i, self.qi_f, v0, vf, a0, af, self.tempo_trajetoria)
            self.pub.publish(traj)

        # Incrementando o contador.
        self.contador +=1


    def get_link_callback(self,msg):

        # Callback destinado a recuperar a pose do executor pelo ground truth. #

        position = msg.pose[-1].position
        self.position = [position.x,position.y,position.z]
        
        orientation = msg.pose[-1].orientation
        orientation = [orientation.x,orientation.y,orientation.z,orientation.w]
        self.orientation = list(euler_from_quaternion(orientation))

        velocity = msg.twist[-1].linear
        self.velocity = [velocity.x,velocity.y,velocity.z]

        angular = msg.twist[-1].angular
        self.angular = [angular.x,angular.y,angular.z]

        # -------------------------------------------------------------------------------------------

        # Adicionando a nova amostra de velocidade
        self.velocity_samples.append(self.velocity)

        # Calculando a diferença entre as velocidades consecutivas
        velocity_diffs = np.diff(self.velocity_samples, axis=0) / 1000

        # calculando médias de velocidade
        self.velocity_mean = np.mean(self.velocity_samples,axis=0)

        # Calculando a aceleração média
        if len(velocity_diffs)!=0:
            acceleration = np.mean(velocity_diffs, axis=0)
            self.acc.append(acceleration.tolist())

        self.vel.append(self.velocity_mean)
        self.pos.append(self.position)
        
        # Filtro para salvar os dados de aceleração
        if len(self.velocity_samples) == 30:
            self.velocity_samples.pop(0)

        if self.flag_dados and self.block:
            print('Salvando os dados.')
            with open("d_pos.txt", 'w') as file:
                for item in self.pos:
                    line = ','.join(map(str, item))
                    file.write(line + '\n')
            with open("d_vel.txt", 'w') as file:
                for item in self.vel:
                    line = ','.join(map(str, item))
                    file.write(line + '\n')
            with open("d_acc.txt", 'w') as file:
                for item in self.acc:
                    line = ','.join(map(str, item))
                    file.write(line + '\n')
            self.block = False

if __name__ == '__main__':
    try:
        # Criando o objeto e iniciando o nó
        init_compare = CompareTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
