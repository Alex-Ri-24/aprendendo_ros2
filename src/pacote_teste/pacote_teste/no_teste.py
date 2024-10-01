import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

def main(args=None):
    # Inicializa o processo
    rclpy.init(args=args)
   
    # Controi o nó
    node = Node('no_simples2')

    # Define o nível do logger
    #logger = node.get_logger()
    #logger.set_level(LoggingSeverity.INFO)

    # Algumas impressões e exemplos de uso do logger
    node.get_logger().info('Pronto')
    node.get_logger().debug ('OPS')
    node.get_logger().info  ('Sera')
    node.get_logger().warn  ('Exemplo')
    node.get_logger().error ('Hoje')
    node.get_logger().fatal ('Teste')
    node.get_logger().info('Finalizando')

    # Destroi o nó
    node.destroy_node()

    # Finaliza o processo
    rclpy.shutdown()


if __name__ == '__main__':
    main()    
