import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3

import numpy as np
import tf_transformations

from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class MeuNo(Node):

    def __init__(self):
        # Definir o nome do nó
        super().__init__('VFH')
        self.get_logger().info('Definido o nome do nó para "VFH"')

        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.get_logger().info('Definindo o subscriber do laser: "/scan"')
        self.laser = None
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)

        self.get_logger().info('Definindo o subscriber de odometria: "/odom"')
        self.pose = None
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)

        self.get_logger().info('Definindo o publisher de controle do robo: "/cmd_vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Definir objetivo de navegação
        self.target_position = (9.0, 9.0)  # Posição final desejada
        self.safe_radius = 2.0  # Raio de segurança para obstáculos
        self.min_distance_front = 2.4  # Distância mínima para detectar a parede à frente

        # Raio de parada (quando estiver perto do alvo)
        self.stop_radius = 0.5  # Raio dentro do qual o robô vai parar

        # Velocidade máxima
        self.max_linear_speed = 0.3
        self.max_angular_speed = 1.0
        self.fast_turn_speed = 1.5  # Velocidade de giro rápido

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def calculate_target_angle(self):
        """Calcula o ângulo entre a posição do robô e o objetivo"""
        if self.pose is None:
            return None
        
        # Posição atual do robô
        robot_x = self.pose.position.x
        robot_y = self.pose.position.y

        # Posição do objetivo
        target_x, target_y = self.target_position

        # Calcular o ângulo desejado
        delta_x = target_x - robot_x
        delta_y = target_y - robot_y
        return np.arctan2(delta_y, delta_x)

    def calculate_current_angle(self):
        """Calcula o ângulo atual do robô"""
        if self.pose is None:
            return None
        
        orientation_q = self.pose.orientation
        _, _, yaw = tf_transformations.euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        return yaw

    def obstacle_in_front(self):
        """Verifica se há uma 'parede' ou obstáculo diretamente à frente"""
        if self.laser is None:
            return False

        # Verifica a distância no setor frontal (entre 80 e 100 graus do LIDAR)
        distancia_frente = min(self.laser[80:100])

        # Verifica se há uma parede ou obstáculo próximo
        return distancia_frente < self.min_distance_front

    def distance_to_target(self):
        """Calcula a distância entre o robô e a posição final desejada"""
        if self.pose is None:
            return float('inf')

        # Posição atual do robô
        robot_x = self.pose.position.x
        robot_y = self.pose.position.y

        # Posição do objetivo
        target_x, target_y = self.target_position

        # Calcular a distância Euclidiana entre a posição atual e o alvo
        return np.sqrt((target_x - robot_x)**2 + (target_y - robot_y)**2)

    def run(self):
        rclpy.spin_once(self)

        self.get_logger().info('Começando VFH :)')

        while rclpy.ok():
            rclpy.spin_once(self)

            if self.laser is None or self.pose is None:
                continue

            # Criar uma mensagem de controle de movimento
            cmd_vel = Twist()

            # Verificar se o robô está perto do alvo
            distancia_para_objetivo = self.distance_to_target()
            if distancia_para_objetivo < self.stop_radius:
                self.get_logger().info("Alvo alcançado, parando o robô.")
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
                self.pub_cmd_vel.publish(cmd_vel)
                continue

            # Verificar se há obstáculo à frente (parede)
            if self.obstacle_in_front():
                self.get_logger().info("Parede detectada à frente, girando rapidamente...")

                # Gira rapidamente para desviar da "parede"
                cmd_vel.linear.x = 0.0  # Parar o movimento linear
                cmd_vel.angular.z = self.fast_turn_speed  # Giro rápido para desviar do obstáculo
            else:
                self.get_logger().info("Caminho livre! Avançando...")

                # Quando não houver obstáculos, ajusta a direção para o objetivo
                target_angle = self.calculate_target_angle()
                current_angle = self.calculate_current_angle()

                if target_angle is not None and current_angle is not None:
                    angle_diff = target_angle - current_angle

                    # Ajustar a velocidade angular apenas quando estiver se movendo reto
                    if np.abs(angle_diff) > 0.1:  # Ajuste para pequenos desvios
                        cmd_vel.angular.z = np.clip(angle_diff, -self.max_angular_speed, self.max_angular_speed)
                    else:
                        cmd_vel.angular.z = 0.0  # Não corrige a rotação

                # Mover-se em linha reta
                cmd_vel.linear.x = self.max_linear_speed

            # Publicar o comando de movimento
            self.pub_cmd_vel.publish(cmd_vel)

    # Destrutor do nó
    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

# Função principal
def main(args=None):
    rclpy.init(args=args)
    node = MeuNo()
    try:
        node.run()
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
