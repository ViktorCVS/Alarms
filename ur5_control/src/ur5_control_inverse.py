#!/usr/bin/env python3

import rospy                                                                 # rospy para utilizar funções do ROS.

from gazebo_msgs.msg import LinkStates                                       # LinkStates para ler o Ground Truth do Gazebo para cada junta, incluindo o executor.
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint        # JointTrajectory para enviar o comando de trajetória e JointTrajectoryPoint para gerar o ponto da trajetória.

import random                                                                # random para escolher aleatoriamente um ângulo para cada junta.
import numpy as np                                                           # numpy para trabalho matricial.
from math import sin,cos,pi, ceil, inf                                       # math para uso das funções seno, cosseno e teto, além do valor de pi e infinito.
from tf.transformations import euler_from_quaternion                         # euler_from_quaternion para converter a orientação de quaternion para euler.

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

        # Variáveis auxiliares para caracterização do erro.
        self.erro_max = -inf
        self.erro_min = inf
        self.erro_medio = 0

        #
        self.margem = 85

        # Contador de iteração.
        self.contador = 0

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

        # ----- Definindo as medidas estruturais do robô UR5, considerando ajustes de simulação e o sistema de coordenadas. -----

        d1 = 0.08915895487108214+0.1000000368892521

        a2  = 0.424994
        d2 = -0.002161932

        a3 = 0.392236
        d3 = 0.109051

        de = 0.004635303

        d4 = 0.09464348

        dx = 0.001118318

        d5 = 0.0823 

        # ----------------------------------------------------------------------------------------------------------------------- 


        # ----- Escolhendo aleatóriamente uma posição para o executor, respeitando as limitações físicas para que não haja colisão/autocolisão. -----

        max_amplitude = (a2+a3)*self.margem/100

        while True:

            x_executor = random.uniform(-max_amplitude,max_amplitude)
            y_executor = random.uniform(-max_amplitude,max_amplitude)
            z_executor = random.uniform(-max_amplitude,max_amplitude)

            if x_executor**2+y_executor**2+z_executor**2 < max_amplitude**2:

                break

        # -------------------------------------------------------------------------------------------------------------------------------------------

        # Montando a mensagem do ponto de referência com base no valor das juntas.
        point.positions = [j1, j2, j3, j4, j5, j6]

        # Definindo o tempo para que a trajetória seja realizada
        point.time_from_start = rospy.Duration(self.tempo_trajetoria)

        # Adicionando a mensagem de ponto para a mensagem da trajetória.
        traj.points.append(point)





        # Montando as medidas estruturais como lista para automação.
        d =    [d1 ,0    ,d2  ,d3   ,d4+de ,d5   ]
        a =    [0  ,0    ,-a2 ,-a3  ,dx    ,0    ]
        alfa = [0  ,pi/2 ,0   ,0    ,pi/2  ,-pi/2]
        j =    [j1 ,j2   ,j3  ,j4   ,j5    ,j6   ]


        # Laço de repetição para multiplicar cada T_i e encontrar a transformação final T com os valores das medidas estruturais e ângulos de cada junta
        i=0
        T=1
        while(i<len(d)):
            T *= self.T_link(j[i],alfa[i],a[i],d[i])
            i+=1
    
        # Encontrando a nova posição pela quarta coluna e três primeiras linhas da matrix T.
        new_position = [-T[0,3],-T[1,3],T[2,3]]

        # Nova pose para previsão atual.
        self.poses[1] = new_position

        # Calculando o erro entre o ground truth e o cálculo matricial.
        Erro = [100*abs(self.position[0]-self.poses[0][0]),100*abs(self.position[1]-self.poses[0][1]),100*abs(self.position[2]-self.poses[0][2])]

        # Formatação das mensagens no terminal, indicando o número da iteração, posição pelo grounth truth e pelo cálculo matricial, além do erro
        # e o valor máximo que esse erro chegou em % discreta de 0.5 em 0.5%.
        if self.contador != 0 :
            
            print("-------------------------------------------------------------------------")
            print(f"Iteração número: {self.contador}\n")
            print(f"Posição [DH]:\nx: {self.poses[0][0]}\ny: {self.poses[0][1]}\nz: {self.poses[0][2]}\n")       #print("pos[k+1]: ",new_position)
            print(f"Posição [GT]:\nx: {self.position[0]}\ny: {self.position[1]}\nz: {self.position[2]}\n")
            print(f"Erro:\nx: {Erro[0]}\ny: {Erro[1]}\nz: {Erro[2]}\n")

            limite = ceil(max(Erro))

            if(limite - max(Erro) > 0.5):
                limite -= 0.5

            if Erro[0]<limite and Erro[1]<limite and Erro[2]<limite:
                print(f"Todos os erros absolutos inferiores a {limite}%")
            if max(Erro)>self.erro_max:
                self.erro_max = max(Erro)
            if min(Erro)<self.erro_min:
                self.erro_min = min(Erro)
            self.erro_medio += (Erro[0]+Erro[1]+Erro[2])/3

            print(f"Erro máximo: {self.erro_max}%")
            print(f"Erro mínimo: {self.erro_min}%")
            print(f"Erro médio: {self.erro_medio/self.contador}%")
            
            print()

        # Enviando a trajetória efetivamente.
        self.pub.publish(traj)

        # A nova posição se torna antiga.
        self.poses[0] = self.poses[1]

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
