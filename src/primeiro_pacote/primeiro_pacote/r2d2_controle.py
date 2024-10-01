import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

import numpy
import tf_transformations

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class R2D2(Node):

    def __init__(self):
        super().__init__('R2D2')
        self.get_logger().debug('Definido o nome do nó para "R2D2"')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().debug('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().debug('Definindo o subscriber de odometria: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().debug('Definindo o publisher de controle do robo: "/cmd_vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        
        #PID
        self.p_gain = 0.020
        self.i_gain = 0.0015
        self.d_gain = 0.005
        self.integral = 0.0
        self.old_error = 0.0

        self.distancia_objetivo = 2.3

        self.ir_para_frente = Twist(linear=Vector3(x=0.4, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.virar_suave = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.2))  # Giro suave

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def run(self):
        rclpy.spin_once(self)

        cmd = Twist()
        cmd.linear.x = 0.4
        cmd.angular.z = 0.0
        self.get_logger().info('Começando')
        self.pub_cmd_vel.publish(cmd)  # Começa andando para frente

        while rclpy.ok():
            rclpy.spin_once(self)

            if self.laser:
                # Usando a mediana ao invés da média
                distancia_direita = numpy.median(self.laser[0:10])

                # Verifica se a distância direita já é igual a 2 metros
                if abs(distancia_direita - self.distancia_objetivo) <= 0.1:
                    self.get_logger().info('Distância correta da parede, andando reto...')
                    cmd.angular.z = 0.0
                else:
                    # Fazendo o ajuste da velocidade angular Z
                    error = self.distancia_objetivo - distancia_direita
                    self.integral += error
                    dif_erro = error - self.old_error
                    self.old_error = error

                    power = self.p_gain * error + self.i_gain * self.integral + self.d_gain * dif_erro

                    cmd.angular.z = power

                self.pub_cmd_vel.publish(cmd)

                # Verificar se há parede na frente
                distancia_frente = min(self.laser[80:100])

                if distancia_frente < 2.4:
                    self.get_logger().info('Parede detectada à frente, girando rapidamente...')
                    # Definir giro rápido
                    cmd.linear.x = 0.0  # Para o movimento linear
                    cmd.angular.z = 1.5  # Aumentar para um giro rápido
                    self.pub_cmd_vel.publish(cmd)

                    # Continuar girando até que o caminho à frente esteja livre
                    while distancia_frente < 2.4:
                        rclpy.spin_once(self)
                        distancia_frente = min(self.laser[80:100])

                    self.get_logger().info('Caminho livre, voltando a andar para frente...')
                    cmd.linear.x = 0.4  # Voltar a andar para frente
                    cmd.angular.z = 0.0  # Parar o giro
                    self.pub_cmd_vel.publish(cmd)

    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')


def main(args=None):
    rclpy.init(args=args)
    node = R2D2()
    try:
        while rclpy.ok():
            node.run()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()