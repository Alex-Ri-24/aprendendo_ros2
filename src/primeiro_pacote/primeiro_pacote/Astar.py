import numpy as np
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

# Função para calcular a distância Euclidiana (h(n))
def distancia_eucli(posicao, objetivo):
    return np.sqrt((posicao[0] - objetivo[0])**2 + (posicao[1] - objetivo[1])**2)

# Função para calcular a probabilidade de ocupação Pocc(n)
def probabilidade_ocupacao(matrix, posicao):
    raio = 3  # Raio para calcular a proximidade dos obstáculos
    ocupacao = 0
    for a in range(-raio, raio + 1):        # Eixo X
        for b in range(-raio, raio + 1):    # Eixo Y 
            vizinho = (posicao[0] + a, posicao[1] + b)
            if 0 <= vizinho[0] < matrix.shape[0] and 0 <= vizinho[1] < matrix.shape[1]:
                ocupacao += 1 - matrix[vizinho]  # Quanto mais próximo de obstáculos, maior Pocc(n)
    
    max_ocupacao = (2 * raio + 1) ** 2  # Máximo de células a considerar no raio
    return ocupacao / max_ocupacao

# Algoritmo A*
def A_Star(matrix, inicio, objetivo):
    # Fila de nós a serem explorados
    fila = [(0, inicio)]  # Contém (custo_prioridade, nó)
    
    # Servem para guardar o custo e o caminho
    custos = {inicio: 0}
    caminho = {inicio: None}
    direcoes = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, 1), (1, -1), (1, 1), (-1, -1)]
    
    while fila:
        # Encontra o menor valor de f(n))
        _, atual = min(fila, key=lambda x: x[0])
        fila.remove((_, atual))
        
        # Se atingiu o objetivo, para a busca
        if atual == objetivo:
            break
        
        for direcao in direcoes:
            vizinho = (atual[0] + direcao[0], atual[1] + direcao[1])
            # Verifica se o vizinho está dentro dos limites da matriz e se é um espaço livre
            if 0 <= vizinho[0] < matrix.shape[0] and 0 <= vizinho[1] < matrix.shape[1] and matrix[vizinho] == 1:
                novo_custo = custos[atual] + distancia_eucli(atual, vizinho)
                p_ocupacao = probabilidade_ocupacao(matrix, vizinho)
                
                # Atualiza se o novo custo for menor
                if vizinho not in custos or novo_custo < custos[vizinho]:
                    custos[vizinho] = novo_custo
                    prioridade = (novo_custo * p_ocupacao) + distancia_eucli(vizinho, objetivo) # f(n) = g(n) * Pocc(n) + h(n)
                    fila.append((prioridade, vizinho))
                    caminho[vizinho] = atual

    caminho_final = []
    atual = objetivo
    while atual is not None:
        caminho_final.append(atual)
        atual = caminho.get(atual)
    
    caminho_final.reverse()
    return caminho_final

class MeuNo(Node):
    def __init__(self):
        super().__init__('Astar_Controller')
        self.get_logger().info('Nó "Astar_Controller" inicializado.')
        
        self.pose = None
        self.start = None       # Posição ainda não definida
        self.goal = (150, 250)  # Posição do objetivo (na escala da matriz)
        self.path = None
        self.current_target_index = 0
        self.target_tolerance = 0.2
        self.angle_tolerance = 0.1  # Tolerância angular para a rotação

        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.listener_callback_odom, 10)

        # Carrega o mapa
        pgmf = open('src/my_map2.pgm', 'rb')
        self.matrix = plt.imread(pgmf)
        pgmf.close()
        self.matrix = 1.0 * (self.matrix > 250)  # Converte para binário (0: obstáculo, 1: livre)

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose
        if self.start is None:
            self.start = self.world_to_grid(self.pose.position.x, self.pose.position.y)  # Convertendo para a matriz
            self.get_logger().info(f'Posição inicial definida: {self.start}')
            self.path = self.generate_path()
            self.visualizar_caminho()
            self.transf_coord(self.path)

    def world_to_grid(self, x, y):
        grid_x = int((y / 0.05) + 200)  # Converte para a célula da matriz, com offset de 200
        grid_y = int((x / 0.05) + 200)  # Converte para a célula da matriz, com offset de 200
        return (grid_x, grid_y)

    def grid_to_world(self, grid):
        x = (grid[1] - 200) * 0.05  # Transforma para coordenadas do mundo
        y = (grid[0] - 200) * 0.05  # Transforma para coordenadas do mundo
        return x, y

    def generate_path(self):
        self.get_logger().info('Gerando caminho com A*...')
        return A_Star(self.matrix, self.start, self.goal)

    def get_robot_position(self):
        return self.pose.position.x, self.pose.position.y

    def calculate_angle_to_target(self, target):
        current_x, current_y = self.get_robot_position()
        target_x, target_y = target
        return np.arctan2(target_y - current_y, target_x - current_x)

    def visualizar_caminho(self):
        if self.path is not None:
            # Cria uma cópia do mapa para exibir o caminho
            mapa_com_caminho = np.copy(self.matrix)
            for cell in self.path:
                mapa_com_caminho[cell] = 0.5  # Marca o caminho no mapa original
            
            # Exibe o mapa com o caminho
            plt.imshow(mapa_com_caminho, interpolation='nearest', cmap='gray')
            plt.title("Caminho Encontrado usando A*: ")
            plt.show()

    def transf_coord(self, caminho):
        caminho_simulacao = []
        for y, x in caminho:
            # Centralizar as coordenadas (0,0) do mapa real para (0,0) da simulação
            centralizado_x = x - 200
            centralizado_y = y - 200
            
            # Ajustar a escala para a simulação (0-400 -> -10 a 10)
            sim_x = (centralizado_x / 200) * 10  # 200 porque metade de 400 equivale a 10 unidades
            sim_y = (centralizado_y / 200) * -10  # Invertido para ajustar a rotação
            
            # Adicionar a coordenada transformada
            caminho_simulacao.append((sim_x, sim_y))
        self.caminho_simulacao = caminho_simulacao

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            if self.pose and self.path and self.current_target_index < len(self.path):
                target = self.grid_to_world(self.path[self.current_target_index])
                robot_x, robot_y = self.get_robot_position()
                distance = np.sqrt((robot_x - target[0])**2 + (robot_y - target[1])**2)

                if distance <= self.target_tolerance:
                    self.current_target_index += 1
                else:
                    twist = Twist()
                    
                    # Calcula o ângulo necessário para virar para o próximo ponto
                    angle_to_target = self.calculate_angle_to_target(target)
                    angle_diff = angle_to_target - self.pose.orientation.z
                    
                    # Ajusta a velocidade angular para virada
                    if abs(angle_diff) > self.angle_tolerance:
                        twist.angular.z = 0.5 * np.sign(angle_diff)
                    else:
                        twist.linear.x = 0.1  # Move para frente

                    # Se chegou no final do caminho, parar
                    if self.current_target_index >= len(self.path):
                        twist.linear.x = 0
                        twist.angular.z = 0
                        self.get_logger().info('cheguei')
                    
                    self.pub_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MeuNo()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
