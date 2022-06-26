"""

AStar search

author: Ashwin Bose (@atb033)

"""
import copy
import heapq
import math


class ANode():
    def __init__(self, node, parent_node, g_score, h_score):
        self.node = node
        self.parent_node = parent_node
        self.g_score = g_score
        self.h_score = h_score

    def get_f_score(self):
        return self.g_score + self.h_score


class AStar():
    def __init__(self):
        self.roadmap = None
        self.start_node = None
        self.goal_node = None

    def get_heuristic_score(self, current_node):
        dx = abs(current_node.x - self.goal_node[0])
        dy = abs(current_node.y - self.goal_node[1])
        return math.sqrt(dx ** 2 + dy ** 2)

    def search(self, roadmap, start_node, goal_node):
        self.roadmap = roadmap
        self.start_node = ANode(start_node, None, 0, 0)

        open_set = list()
        closed_set = list()

        heapq.heappush(open_set, (0, start_node))

        while open_set:
            current_f_score, current_node = heapq.heappop(open_set)

            open_set.extend(current_node.edge)

            for next_weight, next_node in current_node.edge:
                pass


