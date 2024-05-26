from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import numpy as np

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path = self.load_path('paths/path.csv')  # Carregar o caminho salvo
        self.current_pose = None
        self.current_goal_index = 0  # Índice do ponto atual no caminho
        self.obstacle_distance = 0.5  # Distância mínima para detectar um obstáculo

    def laser_callback(self, msg):
        self.laser = msg.ranges

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.follow_path()  # Chamar follow_path a cada atualização de odometria

    def load_path(self, file_path):
        try:
            path = np.loadtxt(file_path, delimiter=",")
            self.get_logger().info(f'Path loaded from {file_path}')
            return path
        except Exception as e:
            self.get_logger().error(f'Failed to load path: {e}')
            return None

    def convert_coordinates_back(self, map_x, map_y, resolution=0.05, origin=(200, 200)):
        # Converte as coordenadas do mapa de volta para as coordenadas do Gazebo, considerando rotação de 90 graus
        gazebo_x = (map_x - origin[1]) * resolution  # Inverter x e y, corrigir sinal
        gazebo_y = (origin[0] - map_y) * resolution  # Corrigir sinal
        self.get_logger().info(f'Converting Map Coordinates ({map_x}, {map_y}) to Gazebo Coordinates ({gazebo_x}, {gazebo_y})')
        return gazebo_x, gazebo_y

    def is_obstacle_ahead(self):
        if self.laser is not None:
            front_ranges = self.laser[80:100]  # Considerar a faixa frontal do sensor
            min_distance = min(front_ranges)
            self.get_logger().info(f'Minimum obstacle distance: {min_distance}')
            return min_distance < self.obstacle_distance
        return False

    def follow_path(self):
        if self.path is None or self.path.size == 0 or self.current_pose is None or self.current_goal_index >= len(self.path):
            self.get_logger().info('Path is None or current_pose is None or path completed')
            return

        # Extraindo posição e orientação atuais
        position = self.current_pose.position
        orientation = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        goal_map = self.path[self.current_goal_index]
        goal_x, goal_y = self.convert_coordinates_back(goal_map[1], goal_map[0])
        dx = goal_x - position.x
        dy = goal_y - position.y
        distance = np.sqrt(dx ** 2 + dy ** 2)
        angle_to_goal = np.arctan2(dy, dx)

        self.get_logger().info(f'Current Position: x={position.x}, y={position.y}, yaw={yaw}')
        self.get_logger().info(f'Goal Position: x={goal_x}, y={goal_y}')
        self.get_logger().info(f'Distance to Goal: {distance}, Angle to Goal: {angle_to_goal}')

        # Controlando o robô para seguir o caminho
        cmd_vel = Twist()
        if self.is_obstacle_ahead():
            self.get_logger().info('Obstacle detected ahead!')
            # Se houver um obstáculo, o robô deve parar e ajustar a direção
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.5  # Girar para a direita
        else:
            if distance > 0.1:
                if abs(angle_to_goal - yaw) > 0.1:  # Se o erro angular for significativo, primeiro alinhe o robô
                    cmd_vel.linear.x = 0.0
                    cmd_vel.angular.z = 1.0 * (angle_to_goal - yaw)  # Ajuste da velocidade angular
                else:
                    cmd_vel.linear.x = 0.2  # Velocidade linear reduzida para maior precisão
                    cmd_vel.angular.z = 0.5 * (angle_to_goal - yaw)  # Ajuste da velocidade angular durante o movimento
            else:
                self.current_goal_index += 1
                self.get_logger().info(f'Moving to next waypoint: {self.current_goal_index}')

        self.pub_cmd_vel.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    robot_control = RobotControl()
    rclpy.spin(robot_control)
    robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
