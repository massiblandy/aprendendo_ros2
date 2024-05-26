from .robot import R2D2
import rclpy
import numpy as np
import math
from geometry_msgs.msg import Twist, Vector3, PoseWithCovarianceStamped
import tf_transformations
from matplotlib import pyplot as plt
import heapq
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class Node:
    def __init__(self, position, parent, g=0, h=0, f=0):
        self.position = position
        self.parent = parent
        self.g = g
        self.h = h
        self.f = f

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def astar(matrix, start, end):
    start_node = Node(start, None)
    end_node = Node(end, None)

    open_list = []
    closed_list = []

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            while current_node != start_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)]

        for next in neighbors:
            if next[0] > (len(matrix) - 1) or next[0] < 0 or next[1] > (len(matrix[len(matrix)-1]) - 1) or next[1] < 0:
                continue

            if matrix[next[0]][next[1]] == 0:
                continue

            child = Node(next, current_node)

            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = heuristic(child.position, end_node.position)
            child.f = child.g + child.h

            if add_to_open(open_list, child):
                heapq.heappush(open_list, child)
    return None

def add_to_open(open_list, child):
    for i, node in enumerate(open_list):
        if child == node and child.g >= node.g:
            return False
    return True

class RobotControl(R2D2):
    def __init__(self):
        super().__init__()
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pose = None
        self.path = None
        self.path_index = 0

        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback_odom, qos_profile)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.navigation_update)  # Chama a função a cada 0.1 segundos

    def listener_callback_odom(self, msg):
        self.pose = msg.pose.pose

    def navigation_start(self):
        self.ir_para_frente = Twist(linear=Vector3(x=0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.ir_para_tras = Twist(linear=Vector3(x=-0.5, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))
        self.girar_direita = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        self.girar_esquerda = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))
        self.parar = Twist(linear=Vector3(x=0.0, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.0))

        self.curva_direita = Twist(linear=Vector3(x=0.1, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=-0.5))
        self.curva_esquerda = Twist(linear=Vector3(x=0.1, y=0.0, z=0.0), angular=Vector3(x=0.0, y=0.0, z=0.5))

        # Carregar mapa
        pgmf = open('src/map.pgm', 'rb')
        self.matrix = plt.imread(pgmf)
        self.matrix = 1.0 * (self.matrix > 250)
        plt.imshow(self.matrix, interpolation='nearest', cmap='gray')
        plt.show()
        
        # Definir ponto de chegada
        self.end = (43, 383)  # Ajuste conforme necessário

    def navigation_update(self):
        # Verificar se a pose está disponível
        if self.pose is None:
            self.get_logger().error("Atributo 'pose' não está disponível.")
            return

        # Definir resolução do mapa
        resolution = 0.05  # Ajuste conforme necessário

        # Obter a posição atual do robô
        current_position = (int(self.pose.position.y / resolution), int(self.pose.position.x / resolution))

        # Planejar o caminho apenas uma vez no início
        if self.path_index == 0 and self.path is None:
            self.path = astar(self.matrix, current_position, self.end)
            if self.path is None:
                self.get_logger().error("Não foi possível encontrar um caminho.")
                self.pub_cmd_vel.publish(self.parar)
                return

            # Desenhar o caminho no mapa
            plt.imshow(self.matrix, interpolation='nearest', cmap='gray')
            for cell in self.path:
                plt.scatter(x=cell[1], y=cell[0], c='r', s=5)
            plt.show()

        if self.path_index >= len(self.path):
            self.pub_cmd_vel.publish(self.parar)
            return

        next_position = self.path[self.path_index]

        target_x = next_position[1] * resolution
        target_y = next_position[0] * resolution

        roll, pitch, yaw = tf_transformations.euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w
        ])

        dx = target_x - self.pose.position.x
        dy = target_y - self.pose.position.y
        angle_to_target = math.atan2(dy, dx)

        if abs(angle_to_target - yaw) > 0.1:
            if angle_to_target > yaw:
                self.pub_cmd_vel.publish(self.girar_esquerda)
            else:
                self.pub_cmd_vel.publish(self.girar_direita)
        else:
            self.pub_cmd_vel.publish(self.ir_para_frente)
            if abs(dx) < 0.1 and abs(dy) < 0.1:
                self.path_index += 1

def main(args=None):
    rclpy.init(args=args)
    r2d2_control = RobotControl()
    r2d2_control.navigation_start()

    # Loop de atualização contínua
    rclpy.spin(r2d2_control)

    r2d2_control.destroy_node()
    rclpy.try_shutdown()
