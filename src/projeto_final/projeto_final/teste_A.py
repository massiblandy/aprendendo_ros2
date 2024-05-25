import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from matplotlib import pyplot as plt
import numpy as np
import heapq

class AStarPlanner(Node):
    def __init__(self):
        super().__init__('a_star_planner')
        self.publisher = self.create_publisher(Point, 'path', 10)
        self.timer = self.create_timer(1, self.publish_path)

    def publish_path(self):
        start = (309, 54)  # Define o ponto de início
        end = (43, 383)    # Define o ponto de fim
        matrix = self.load_map()  # Carrega o mapa
        path = self.astar(matrix, start, end)  # Calcula o caminho A*
        if path:
            for cell in path:
                point = Point()
                point.x = cell[1]  # Ajusta as coordenadas conforme necessário
                point.y = cell[0]
                self.publisher.publish(point)

    def load_map(self):
        # Carrega o mapa do arquivo map.pgm
        pgmf = open('map.pgm', 'rb')
        matrix = plt.imread(pgmf)
        matrix = 1.0 * (matrix > 250)  # Converte os valores da matriz para binário (0 ou 1)
        return matrix

    def astar(self, matrix, start, end):
        # Cria os nós de início e fim
        start_node = Node(start, None)
        end_node = Node(end, None)

        # Cria listas aberta (todos os nós que ainda não foram abertos) e fechada (contém os nós que já foram analisados)
        open_list = []
        closed_list = []

        # Adiciona o nó de início à lista aberta
        heapq.heappush(open_list, start_node)

        # Loop até encontrar o fim
        while len(open_list) > 0:
            # Pega o nó atual
            current_node = heapq.heappop(open_list)
            closed_list.append(current_node)

            # Checa se alcançamos o objetivo, retorna o caminho
            if current_node == end_node:
                path = []
                while current_node != start_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]  # Retorna o caminho invertido

            # Gera filhos
            (x, y) = current_node.position
            neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1), (x-1, y-1), (x+1, y-1), (x-1, y+1), (x+1, y+1)]  # Nós adjacentes e diagonais

            for next in neighbors:
                # Verifica se está dentro dos limites da matriz
                if next[0] > (len(matrix) - 1) or next[0] < 0 or next[1] > (len(matrix[len(matrix)-1]) - 1) or next[1] < 0:
                    continue

                # Pega valor do mapa
                map_value = matrix[next[0]][next[1]]
                # Checa se é um obstáculo
                if map_value == 0:
                    continue

                # Cria um nó filho
                child = Node(next, current_node)

                # Conferindo se nó filho está na lista já analisada
                if child in closed_list:
                    continue

                # Cria os valores f, g e h dos nós filhos
                child.g = current_node.g + 1
                child.h = self.heuristic(child.position, end_node.position)
                child.f = child.g + child.h

                # Nó filho já está na lista de nós que ainda não foram abertos
                if self.add_to_open(open_list, child):
                    heapq.heappush(open_list, child)
        return None

    # Checa se um nó filho deve ser adicionado à open_list (lista de nós que ainda não foram abertos)
    def add_to_open(self, open_list, child):
        for i, node in enumerate(open_list):
            if child == node:
                if child.g < node.g:
                    open_list[i] = child  # Substitui o nó existente pelo novo com menor custo g
                    return True
                else:
                    return False
        return True

    # Função heurística para calcular h baseada na distância de Manhattan
    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

def main(args=None):
    rclpy.init(args=args)
    a_star_planner = AStarPlanner()
    rclpy.spin(a_star_planner)
    a_star_planner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
