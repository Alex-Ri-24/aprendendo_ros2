import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import tf_transformations
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

# Função de normalização de ângulo
def angle_normalize(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))

# Nó principal
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

        self.get_logger().info('Definindo o publisher de controle do robô: "/cmd_vel"')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_position = (9.0, 9.0)
        self.safe_radius = 1.0
        self.safe_distance = 3.5  # Distância para desvio de obstáculos
        self.alignment_threshold = np.pi / 6

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def get_robot_orientation(self):
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        return yaw

    def get_robot_position(self):
        return self.pose.position.x, self.pose.position.y

    def calculate_distance_to_target(self):
        current_x, current_y = self.get_robot_position()
        target_x, target_y = self.target_position
        return np.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)

    def calculate_target_angle(self):
        current_x, current_y = self.get_robot_position()
        target_x, target_y = self.target_position
        return np.arctan2(target_y - current_y, target_x - current_x)

    def vfh(self):
        if self.laser is None or self.pose is None:
            return None

        # Histograma para obstáculos
        histogram = np.zeros(len(self.laser))

        for i, distance in enumerate(self.laser):
            if 0 < distance < self.safe_distance:  # Se a distância for menor que a segura
                histogram[i] = 1  # Marca como ocupado
            else:
                histogram[i] = 0  # Marca como livre

        # Encontrar setores livres
        free_sectors = np.where(histogram == 0)[0]

        if len(free_sectors) > 0:
            max_angle = None
            max_distance = -1
            max_width = 0
            current_width = 0
            
            # Percorrer os setores livres para identificar o mais largo
            for i in range(len(histogram)):
                if histogram[i] == 0:  # Se o setor é livre
                    current_width += 1  # Aumentar a largura do setor livre
                    if self.laser[i] > max_distance:
                        max_distance = self.laser[i]  # Atualiza a maior distância
                        max_angle = (i - len(self.laser) / 2) * (np.pi / len(self.laser))
                        max_width = current_width  # Atualiza a largura do maior setor
                else:
                    current_width = 0  # Reinicia a largura se encontrar um setor ocupado

            return max_angle  # Retorna o ângulo do setor livre mais distante
        else:
            return None


    def compute_cmd_vel(self, target_angle):
        twist = Twist()
        yaw = self.get_robot_orientation()
        angle_diff = angle_normalize(target_angle - yaw)

        if abs(angle_diff) < self.alignment_threshold:
            twist.linear.x = 0.2  # Reduzir a velocidade linear para 0.2
            twist.angular.z = 0.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.3 if angle_diff > 0 else -0.3

        return twist

    def run(self):
        rclpy.spin_once(self)

        while rclpy.ok():
            rclpy.spin_once(self)

            if self.pose:
                distance_to_target = self.calculate_distance_to_target()
                if distance_to_target <= self.safe_radius:
                    self.get_logger().info('Robô chegou ao destino.')
                    cmd_vel = Twist()  # Parando o robô
                else:
                    target_angle = self.calculate_target_angle()

                    if self.laser:
                        vfh_angle = self.vfh()

                        if vfh_angle is not None:
                            # Se há um ângulo livre, ajusta a direção
                            twist = self.compute_cmd_vel(vfh_angle)
                        else:
                            # Se não há ângulo livre, apenas segue o ângulo do objetivo
                            twist = self.compute_cmd_vel(target_angle)

                self.pub_cmd_vel.publish(twist)

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
