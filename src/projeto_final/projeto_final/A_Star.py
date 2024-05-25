import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import heapq
from matplotlib import pyplot as plt

class AStarPlanner(Node):
    def __init__(self, map_pgm):
        super().__init__('a_star_planner')
        self.publisher = self.create_publisher(Point, 'path', 10)
        self.timer = self.create_timer(1, self.publish_path)
        self.matrix = self.load_map(map_pgm)

    def load_map(self, map_pgm):
        try:
            with open(map_pgm, 'rb') as pgmf:
                matrix = plt.imread(pgmf)
                matrix = np.array(matrix)
                return matrix
        except FileNotFoundError:
            self.get_logger().error(f"Map file '{map_pgm}' not found.")
            return None

    def publish_path(self):
        start = (309, 54)  # Define o ponto de início
        end = (43, 383)    # Define o ponto de fim
        path = self.astar(self.matrix, start, end)
        if path:
            for cell in path:
                point = Point()
                point.x = cell[1]  # Ajusta as coordenadas conforme necessário
                point.y = cell[0]
                point.z = 0.0
                self.publisher.publish(point)

    @staticmethod
    def heuristic(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def astar(matrix, start, end):
        class AStarNode:
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

        start_node = AStarNode(start, None)
        end_node = AStarNode(end, None)

        open_list = []
        closed_list = []

        heapq.heappush(open_list, start_node)

        while len(open_list) > 0:
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

            for next_node in neighbors:
                if next_node[0] > (len(matrix) - 1) or next_node[0] < 0 or next_node[1] > (len(matrix[0]) - 1) or next_node[1] < 0:
                    continue

                map_value = matrix[next_node[0]][next_node[1]]
                if map_value == 0:
                    continue

                child = AStarNode(next_node, current_node)

                if child in closed_list:
                    continue

                child.g = current_node.g + 1
                child.h = AStarPlanner.heuristic(child.position, end_node.position)
                child.f = child.g + child.h

                if AStarPlanner.add_to_open(open_list, child):
                    heapq.heappush(open_list, child)
        return None

    @staticmethod
    def add_to_open(open_list, child):
        for node in open_list:
            if child == node and child.g >= node.g:
                return False
        return True

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) < 2:
        print("Usage: ros2 run projeto_final A_Star /workspaces/aprendendo_ros2/src/projeto_final/projeto_final/map.pgm")
        return
    map_pgm = sys.argv[1]
    a_star_planner = AStarPlanner(map_pgm)
    if a_star_planner.matrix is None:
        print("Failed to load map.")
        return
    rclpy.spin(a_star_planner)
    a_star_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
