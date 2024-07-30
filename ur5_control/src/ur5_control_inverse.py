#!/usr/bin/env python3

import rospy                                                                 # rospy para utilizar funções do ROS.

from gazebo_msgs.msg import LinkStates                                       # LinkStates para ler o Ground Truth do Gazebo para cada junta, incluindo o executor.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint        # JointTrajectory para enviar o comando de trajetória e JointTrajectoryPoint para gerar o ponto da trajetória.

import random                                                                # random para escolher aleatoriamente um ângulo para cada junta.
import numpy as np                                                           # numpy para trabalho matricial.
from math import sin,cos,pi, ceil, inf, acos, asin, atan2, sqrt                                     # math para uso das funções seno, cosseno e teto, além do valor de pi e infinito.
from tf.transformations import euler_from_quaternion                         # euler_from_quaternion para converter a orientação de quaternion para euler.
from scipy.spatial.transform import Rotation as R


class CompareTrajectory():
    def __init__(self):

        # Nomeando o nó.
        rospy.init_node('ur5_compare_trajectory') 

        # Criar um publisher para o tópico de comando de trajetória.
        self.pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
        
        # Criar um subscriber para o tópico de posição das juntas.
        rospy.Subscriber('/gazebo/link_states', LinkStates, self.get_link_callback)

        # Esperar pelo publisher e subscriber se conectarem.
        rospy.sleep(1)

        # Intervalo de tempo para enviar a referência de trajetória.
        tempo_referencia = 5

        # Tempo para que a trajetória seja concluída quando a referência for enviada.
        self.tempo_trajetoria = 2.5

        # Vetor que acumulará a posição prevista para a junta, posição passada e atual.
        self.poses = [[0,0,0],0]

        self.orien = [[0,0,0],0]

        # Variáveis auxiliares para caracterização do erro.
        self.erro_max_junta = -inf
        self.erro_min_junta = inf
        self.erro_medio_junta = 0
        self.erro_medio_junta_parcial = []

        self.erro_max = -inf
        self.erro_min = inf
        self.erro_medio = 0

        self.erro_max_orien = -inf
        self.erro_min_orien = inf
        self.erro_medio_orien = 0

        # Contador de iteração.
        self.contador = 0
        self.inner_contador = 0

        self.j1 = 0
        self.j2 = 0
        self.j3 = 0
        self.j4 = 0
        self.j5 = 0
        self.j6 = 0

        self.J = np.zeros((6,2))

        self.position_anterior = [0,0,0]
        self.orientation_anterior = [0,0,0]

        # Timer para enviar a referência de posição periodicamente.
        rospy.Timer(rospy.Duration(tempo_referencia), self.timer_callback)  


    def timer_callback(self,event): 

        # Função que calcula a trajetória futura, envia a referência para o controlador de posição e compara a previsão com o ground truth. #

        # Iniciando a mensagem de trajetória.
        traj = JointTrajectory()

        # Incluindo o nome das juntas.
        traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Iniciando a mensagem de ponto de referência.
        point = JointTrajectoryPoint()

        # ----- Escolhendo aleatóriamente ângulo para as juntas, respeitando as limitações físicas para que não haja colisão/autocolisão. -----

        if self.contador%3 == 0:

            self.j1 = random.uniform(-pi,pi)
            self.j2 = random.uniform(0,-pi)

            if self.j2 > -pi/2:
                self.j3 = random.uniform(0,-pi/2)
            elif self.j2 < -pi/2:
                self.j3 = random.uniform(0,pi/2)

            self.j4 = random.uniform(0,-pi)

            self.j5 = random.uniform(0,2*pi)

            self.j6 = random.uniform(-pi,pi)

            self.J[0,1] = self.j1
            self.J[1,1] = self.j2
            self.J[2,1] = self.j3
            self.J[3,1] = self.j4
            self.J[4,1] = self.j5
            self.J[5,1] = self.j6

        # -------------------------------------------------------------------------------------------------------------------------------------


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


        # Montando as medidas estruturais como lista para automação.
        d =    [d1 ,0    ,-d2 ,d3    ,d4+de ,d5   ]
        a =    [0  ,0    ,-a2 ,-a3+ax,0     ,0    ]
        alfa = [0  ,pi/2 ,0   ,0     ,pi/2  ,-pi/2]
        j =    [self.j1 ,self.j2   ,self.j3  ,self.j4    ,self.j5    ,self.j6   ]


        if self.contador%3 == 1:

            T = np.matrix([[cos(self.orientation[2])*cos(self.orientation[1]), cos(self.orientation[2])*sin(self.orientation[1])*sin(self.orientation[0])-sin(self.orientation[2])*cos(self.orientation[0]), cos(self.orientation[2])*sin(self.orientation[1])*cos(self.orientation[0])+sin(self.orientation[2])*sin(self.orientation[0]), self.position[0]],
                        [sin(self.orientation[2])*cos(self.orientation[1]), sin(self.orientation[2])*sin(self.orientation[1])*sin(self.orientation[0])+cos(self.orientation[2])*cos(self.orientation[0]), sin(self.orientation[2])*sin(self.orientation[1])*cos(self.orientation[0])-cos(self.orientation[2])*sin(self.orientation[0]), self.position[1]],
                        [-sin(self.orientation[1]), cos(self.orientation[1])*sin(self.orientation[0]), cos(self.orientation[1])*cos(self.orientation[0]), self.position[2]],
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

            if abs(self.J[0,0]-self.tetha_1)>=abs(self.J[0,0]-tetha_1x):
                self.tetha_1 = tetha_1x

            rex = (-self.position[0]*sin(self.tetha_1)+self.position[1]*cos(self.tetha_1)-d3)/d5
            self.tetha_5 = acos(rex)

            if abs(self.tetha_5 - self.J[4,0]) >= abs(-self.tetha_5+2*pi -self.J[4,0]):
                self.tetha_5 = -self.tetha_5+2*pi



            if self.orientation[2] >= 0: alfa = self.orientation[2]-pi
            elif self.orientation[2] < 0: alfa = self.orientation[2]+pi

            T = np.matrix([[cos(alfa)*cos(self.orientation[1]), cos(alfa)*sin(self.orientation[1])*sin(self.orientation[0])-sin(alfa)*cos(self.orientation[0]), cos(alfa)*sin(self.orientation[1])*cos(self.orientation[0])+sin(alfa)*sin(self.orientation[0]), -self.position[0]],
                        [sin(alfa)*cos(self.orientation[1]), sin(alfa)*sin(self.orientation[1])*sin(self.orientation[0])+cos(alfa)*cos(self.orientation[0]), sin(alfa)*sin(self.orientation[1])*cos(self.orientation[0])-cos(alfa)*sin(self.orientation[0]), -self.position[1]],
                        [-sin(self.orientation[1]), cos(self.orientation[1])*sin(self.orientation[0]), cos(self.orientation[1])*cos(self.orientation[0]), self.position[2]],
                        [0, 0, 0, 1]])
            T_i = np.linalg.inv(T)

            self.tetha_6 = atan2((-T_i[1,0]*sin(self.tetha_1)+T_i[1,1]*cos(self.tetha_1))/sin(self.tetha_5),(T_i[0,0]*sin(self.tetha_1)-T_i[0,1]*cos(self.tetha_1))/sin(self.tetha_5))


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

            if abs(self.J[2,0]-self.tetha_3)>=abs(self.J[2,0]-tetha_3x):
                self.tetha_3 = tetha_3x


            self.tetha_2 = atan2(-T14[2,3],-T14[0,3])-asin((a3-ax)*sin(self.tetha_3)/sqrt(T14[0,3]**2+T14[2,3]**2))


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

            if abs(self.J[3,0]-self.tetha_4)>=abs(self.J[3,0]+self.tetha_4):
                self.tetha_4 *= -1

            self.position_anterior[0] = self.position[0]
            self.position_anterior[1] = self.position[1]
            self.position_anterior[2] = self.position[2]

            self.orientation_anterior[0] = self.orientation[0]
            self.orientation_anterior[1] = self.orientation[1]
            self.orientation_anterior[2] = self.orientation[2]

        # Formatação das mensagens no terminal, indicando o número da iteração, posição pelo grounth truth e pelo cálculo matricial, além do erro
        # e o valor máximo que esse erro chegou em % discreta de 0.5 em 0.5%.
        if self.contador%3 == 2:

            # Calculando o erro entre o ground truth e o cálculo matricial.
            Erro = [100*abs(self.J[0,0]-self.tetha_1),100*abs(self.J[1,0]-self.tetha_2),100*abs(self.J[2,0]-self.tetha_3),100*abs(self.J[3,0]-self.tetha_4),100*abs(self.J[4,0]-self.tetha_5),100*abs(self.J[5,0]-self.tetha_6)]

            limite = ceil(max(Erro))

            if(limite - max(Erro) > 0.5):
                limite -= 0.5

            if max(Erro)>self.erro_max_junta:
                self.erro_max_junta = max(Erro)
            if min(Erro)<self.erro_min_junta:
                self.erro_min_junta = min(Erro)
            self.erro_medio_junta += (Erro[0]+Erro[1]+Erro[2])/3
            self.erro_medio_junta_parcial.append((Erro[0]+Erro[1]+Erro[2])/3)

            if len(self.erro_medio_junta_parcial) > 10:
                self.erro_medio_junta_parcial.pop(0)

            Erro_position = [100*abs(self.position_anterior[0]-self.position[0]),100*abs(self.position_anterior[1]-self.position[1]),100*abs(self.position_anterior[2]-self.position[2])]
            Erro_orientation = [100*abs(self.orientation_anterior[0]-self.orientation[0]),100*abs(self.orientation_anterior[1]-self.orientation[1]),100*abs(self.orientation_anterior[2]-self.orientation[2])]

            limite_pos = ceil(max(Erro_position))
            limite_ori = ceil(max(Erro_orientation))

            if(limite_pos - max(Erro_position) > 0.5):
                limite_pos -= 0.5
            
            if(limite_ori - max(Erro_orientation) > 0.5):
                limite_ori -= 0.5
            
            if max(Erro_position)>self.erro_max:
                self.erro_max = max(Erro_position)
            if min(Erro_position)<self.erro_min:
                self.erro_min = min(Erro_position)
            self.erro_medio += (Erro_position[0]+Erro_position[1]+Erro_position[2])/3

            if max(Erro_orientation)>self.erro_max_orien:
                self.erro_max_orien = max(Erro_orientation)
            if min(Erro_orientation)<self.erro_min_orien:
                self.erro_min_orien = min(Erro_orientation)
            self.erro_medio_orien += (Erro_orientation[0]+Erro_orientation[1]+Erro_orientation[2])/3

            print("-----------------------------------------------------------------------------------------------")
            print(f"Iteração número: {self.inner_contador+1}\n")
            print(f"tetha 1 [GT]: {round(self.J[0,0],4):.4f}\t\ttetha 2 [GT]: {round(self.J[1,0],4):.4f}\t\ttetha 3 [GT]: {round(self.J[2,0],4):.4f}")
            print(f"tetha 1 [SG]: {round(self.tetha_1,4):.4f}\t\ttetha 2 [SG]: {round(self.tetha_2,4):.4f}\t\ttetha 3 [SG]: {round(self.tetha_3,4):.4f}")
            print(f"Erro tetha 1: {round(Erro[0],4):.4f}%\t\tErro tetha 2: {round(Erro[1],4):.4f}%\t\tErro tetha 3: {round(Erro[2],4):.4f}%")
            print()
            print(f"tetha 4 [GT]: {round(self.J[3,0],4):.4f}\t\ttetha 5 [GT]: {round(self.J[4,0],4):.4f}\t\ttetha 6 [GT]: {round(self.J[5,0],4):.4f}")
            print(f"tetha 4 [SG]: {round(self.tetha_4,4):.4f}\t\ttetha 5 [SG]: {round(self.tetha_5,4):.4f}\t\ttetha 6 [SG]: {round(self.tetha_6,4):.4f}")
            print(f"Erro tetha 4: {round(Erro[3],4):.4f}%\t\tErro tetha 5: {round(Erro[4],4):.4f}%\t\tErro tetha 6: {round(Erro[5],4):.4f}%")
            print()
            print(f"Todos os erros absolutos de junta inferiores a {limite}%\n")
            print(f"Erro máximo global: {round(self.erro_max_junta,4):.4f}%\tErro mínimo global: {round(self.erro_min_junta,4):.4f}%\tErro médio global : {round(self.erro_medio_junta/self.contador,4):.4f}%")
            print(f"Erro máximo atual : {round(max(Erro),4):.4f}%\tErro mínimo atual : {round(min(Erro),4):.4f}%\tErro médio parcial: {round(sum(self.erro_medio_junta_parcial)/self.contador,4):.4f}%")
            print()
            print(f"Posição indicada pelas juntas:\t\tPosição indicada pela cinemática inversa:")
            print(f"x [GT]: {round(self.position_anterior[0],4):.4f}\t\t\t\tx [SG]: {round(self.position[0],4):.4f}")
            print(f"y [GT]: {round(self.position_anterior[1],4):.4f}\t\t\t\ty [SG]: {round(self.position[1],4):.4f}")
            print(f"z [GT]: {round(self.position_anterior[2],4):.4f}\t\t\t\tz [SG]: {round(self.position[2],4):.4f}")
            print()
            print(f"Orientação indicada pelas juntas:\tOrientação indicada pela cinemática inversa:")
            print(f"x [GT]: {round(self.orientation_anterior[0],4):.4f}\t\t\t\tx [SG]: {round(self.orientation[0],4):.4f}")
            print(f"y [GT]: {round(self.orientation_anterior[1],4):.4f}\t\t\t\ty [SG]: {round(self.orientation[1],4):.4f}")
            print(f"z [GT]: {round(self.orientation_anterior[2],4):.4f}\t\t\t\tz [SG]: {round(self.orientation[2],4):.4f}")
            print()
            print(f"Erro de posição:\t\t\tErro de orientação:")
            print(f"x: {round(Erro_position[0],4):.4f}%\t\t\t\tx: {round(Erro_orientation[0],4):.4f}%")
            print(f"y: {round(Erro_position[1],4):.4f}%\t\t\t\ty: {round(Erro_orientation[1],4):.4f}%")
            print(f"z: {round(Erro_position[2],4):.4f}%\t\t\t\tz: {round(Erro_orientation[2],4):.4f}%")
            print()
            print(f"Todos os erros absolutos de posição inferiores a {limite_pos}%")
            print(f"Todos os erros absolutos de orientação inferiores a {limite_ori}%")
            print()
            print(f"Erro máximo de posição: {round(self.erro_max,4):.4f}%\t\t\tErro máximo de orientação: {round(self.erro_max_orien,4):.4f}%")
            print(f"Erro mínimo de posição: {round(self.erro_min,4):.4f}%\t\t\tErro mínimo de orientação: {round(self.erro_min_orien,4):.4f}%")
            print(f"Erro médio de posição : {round(self.erro_medio/self.contador,4):.4f}%\t\t\tErro médio de orientação : {round(self.erro_medio_orien/self.contador,4):.4f}%")
            print()

#{round(Erro_position[0],4):.4f} {round(Erro_position[1],4):.4f}%
            self.inner_contador += 1

        # Montando a mensagem do ponto de referência com base no valor das juntas.
        if self.contador%3 == 0:
            point.positions = [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6]
            point.time_from_start = rospy.Duration(self.tempo_trajetoria)
            traj.points.append(point)
            self.pub.publish(traj)

        elif self.contador%3 == 1:
            point.positions = [self.tetha_1, self.tetha_2, self.tetha_3, self.tetha_4, self.tetha_5, self.tetha_6]
            point.time_from_start = rospy.Duration(self.tempo_trajetoria)
            traj.points.append(point)
            self.pub.publish(traj)
            
        elif self.contador%3 == 2:
            pass


        if self.contador%3 == 0:
            self.J[0,0] = self.J[0,1]
            self.J[1,0] = self.J[1,1]
            self.J[2,0] = self.J[2,1]
            self.J[3,0] = self.J[3,1]
            self.J[4,0] = self.J[4,1]
            self.J[5,0] = self.J[5,1]

        # Incrementando o contador.
        self.contador +=1


    def get_link_callback(self,msg):

        # Callback destinado a recuperar a posição do executor pelo ground truth. #

        position = msg.pose[-1].position
        self.position = [position.x,position.y,position.z]
        
        orientation = msg.pose[-1].orientation
        orientation = [orientation.x,orientation.y,orientation.z,orientation.w]
        self.orientation = list(euler_from_quaternion(orientation))

    def T_link(self,theta,alfa,a,d):

        # Função que monta a matrix da transformação homogênea entre as juntas. #

        return np.matrix([[cos(theta)          ,-sin(theta)         ,0         ,a           ],
                          [sin(theta)*cos(alfa),cos(theta)*cos(alfa),-sin(alfa),-sin(alfa)*d],
                          [sin(theta)*sin(alfa),cos(theta)*sin(alfa),cos(alfa) ,cos(alfa)*d ],
                          [0                   ,0                   ,0         ,1           ]])
    

 

if __name__ == '__main__':
    try:
        # Criando o objeto e iniciando o nó
        init_compare = CompareTrajectory()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
