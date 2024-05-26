from matplotlib import pyplot as plt
import numpy as np
import heapq

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

pgmf = open('src/map.pgm', 'rb')
matrix = plt.imread(pgmf)
matrix = 1.0 * (matrix > 250)
plt.imshow(matrix, interpolation='nearest', cmap='gray')
plt.show()

start = (202, 205)
end = (309, 54)

path = astar(matrix, start, end)

plt.imshow(matrix, interpolation='nearest', cmap='gray')
if path:
    for cell in path:
        plt.scatter(x=cell[1], y=cell[0], c='r', s=5)
plt.show()
