import numpy as np
import matplotlib.pyplot as plt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
import time

class MeuNo(Node):

    def __init__(self):
        super().__init__('WAVEFRONT')
        self.get_logger().info('Definido o nome do nó para "WAVEFRONT"')

        # Definir subscribers e publishers
        qos_profile = QoSProfile(depth=10)
        self.create_subscription(LaserScan, '/scan', self.listener_callback_laser, qos_profile)
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # Inicializar variáveis
        self.laser = None
        self.pose = None

        # Carregar o mapa
        self.map_matrix = self.load_map()

        # Verificar se o mapa foi carregado corretamente
        if self.map_matrix is not None:
            self.get_logger().info('Mapa carregado com sucesso!')
        else:
            self.get_logger().error('Falha ao carregar o mapa! Verifique o arquivo.')

        # Definir posições iniciais e de destino
        self.start_position = [-8, -8]  # Posição inicial
        self.goal = [0, 0]  # Objetivo final

    def load_map(self):
        """Carrega o mapa do arquivo PGM e normaliza os valores"""
        try:
            pgmf = open('src/my_map2.pgm', 'rb')
            matrix = plt.imread(pgmf)

            # Normalizar o mapa: 0 = livre, 1 = obstáculo
            matrix = 1.0 * (matrix < 250)
            return matrix
        except Exception as e:
            self.get_logger().error(f'Erro ao carregar o mapa: {e}')
            return None

    def listener_callback_laser(self, msg):
        self.laser = msg.ranges

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def wavefront_path(self, start, goal):
        if self.map_matrix is None:
            self.get_logger().error("Mapa não carregado! Impossível executar o wavefront.")
            return []

        matrix = self.map_matrix.copy()  # Copia a matriz para evitar alterações indesejadas
        path = []
        
        # Usar a posição do objetivo na matriz para iniciar o wavefront
        goal_position = [goal[0], goal[1]]
        matrix[goal_position[0]][goal_position[1]] = 0
        queue = [goal_position]

        # Criar a matriz de wavefront
        while queue:
            current = queue.pop(0)
            current_value = matrix[current[0]][current[1]]
            neighbors = self.get_neighbors(current)

            for neighbor in neighbors:
                if matrix[neighbor[0]][neighbor[1]] == 0:  # Se célula livre
                    matrix[neighbor[0]][neighbor[1]] = current_value + 1
                    queue.append(neighbor)

        # Encontrar o caminho de menor custo
        current = start
        self.get_logger().info('antes')
        while current != goal:
            neighbors = self.get_neighbors(current)
            valid_neighbors = [n for n in neighbors if self.is_valid(n)]
            self.get_logger().info('depois')
            if not valid_neighbors:
                self.get_logger().error("Não há caminho válido para o objetivo.")
                return path  # Retornar caminho vazio se não houver vizinhos válidos
            
            # Suavização: calcular o custo levando em consideração a distância até o objetivo
            self.get_logger().info('cu')
            next_cell = min(valid_neighbors, key=lambda x: matrix[x[0]][x[1]] + self.get_distance(x, goal))
            self.get_logger().info('não é aqui')

            # Verificar se está próximo a obstáculos
            if self.is_near_obstacle(next_cell):
                self.get_logger().warn(f"Próximo demais de um obstáculo em {next_cell}, ajustando caminho...")
                next_cell = self.avoid_obstacle(valid_neighbors, matrix)

            # Adicionar a célula ao caminho
            path.append(next_cell)
            current = next_cell
        return path

    def get_neighbors(self, pos):
        # Define as possíveis posições vizinhas (acima, abaixo, esquerda, direita)
        neighbors = [
            [pos[0] - 1, pos[1]],  # Vizinho acima
            [pos[0] + 1, pos[1]],  # Vizinho abaixo
            [pos[0], pos[1] - 1],  # Vizinho à esquerda
            [pos[0], pos[1] + 1]   # Vizinho à direita
        ]
        
        # Filtra os vizinhos para garantir que estejam dentro dos limites válidos da matriz
        return [n for n in neighbors if self.is_valid(n)]

    def is_valid(self, pos):
        return 0 <= pos[0] < self.map_matrix.shape[0] and 0 <= pos[1] < self.map_matrix.shape[1]

    def get_distance(self, pos1, pos2):
        """Calcula a distância entre duas posições"""
        return np.linalg.norm(np.array(pos1) - np.array(pos2))

    def is_near_obstacle(self, pos):
        """Verifica se a célula está muito próxima de um obstáculo"""
        neighbors = self.get_neighbors(pos)
        # Suavização: Permitir passar mais próximo de obstáculos, apenas evitando o contato direto
        for neighbor in neighbors:
            if self.map_matrix[neighbor[0]][neighbor[1]] == 1:  # Obstáculo direto
                return True
        return False

    def avoid_obstacle(self, neighbors, matrix):
        """Ajusta o caminho para evitar passar próximo de obstáculos"""
        safe_neighbors = [n for n in neighbors if not self.is_near_obstacle(n)]
        if safe_neighbors:
            return min(safe_neighbors, key=lambda x: matrix[x[0]][x[1]])
        return neighbors[0]  # Se não houver rota segura, seguir o caminho original

    def move_robot(self, path):
        """Movimenta o robô ao longo do caminho encontrado"""
        twist = Twist()
        for step in path:
            self.get_logger().info(f'Movendo para {step}')
            target_x, target_y = step
            current_x = self.pose.position.x
            current_y = self.pose.position.y

            distance = np.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
            if distance > 0.1:  # Distância mínima para considerar que o robô se moveu
                twist.linear.x = 0.2  # Velocidade linear
                twist.angular.z = self.calculate_angular_velocity(target_x, target_y)  # Calcular rotação
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            
            self.pub_cmd_vel.publish(twist)
            time.sleep(0.5)

    def calculate_angular_velocity(self, target_x, target_y):
        """Calcula a velocidade angular necessária para alinhar o robô com o alvo"""
        delta_x = target_x - self.pose.position.x
        delta_y = target_y - self.pose.position.y
        angle_to_target = np.arctan2(delta_y, delta_x)
        current_orientation = self.pose.orientation
        current_angle = np.arctan2(current_orientation.z, current_orientation.w)  # Simplificação para orientação
        angular_velocity = angle_to_target - current_angle
        return angular_velocity

    def run(self):
        rclpy.spin_once(self)

        path = self.wavefront_path(self.start_position, self.goal)
        self.get_logger().info(f'Path encontrado: {path}')

        # Mover o robô ao longo do caminho
        self.move_robot(path)

        while rclpy.ok():
            rclpy.spin_once(self)

    def __del__(self):
        self.get_logger().info('Finalizando o nó! Tchau, tchau...')

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
