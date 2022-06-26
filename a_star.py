"""

AStar search

author: Ashwin Bose (@atb033)

"""
import copy
import heapq
import math


class ANode():
    def __init__(self, node, g_score, h_score):
        self.node = node
        self.g_score = g_score
        self.h_score = h_score

    def get_f_score(self):
        return self.g_score + self.h_score

    def __lt__(self, other):
        return self.get_f_score() < other.get_f_score()


class AStar():
    def __init__(self):
        self.roadmap = None
        self.start_node = None
        self.goal_node = None

    def get_heuristic_score(self, current_node):
        dx = abs(current_node.x - self.goal_node.x)
        dy = abs(current_node.y - self.goal_node.y)
        return math.sqrt(dx ** 2 + dy ** 2)

    def search(self, roadmap, start_node, goal_node):
        self.roadmap = roadmap
        self.start_node = ANode(start_node, 0, 0)
        self.goal_node = goal_node

        open_set = list()
        closed_set = list()

        heapq.heappush(open_set, self.start_node)

        while open_set:
            current_node = heapq.heappop(open_set)
            closed_set.append(current_node.node)

            for next_weight, next_node in current_node.node.edge:
                if next_node.x == goal_node.x and next_node.y == goal_node.y:
                    return current_node.g_score + next_weight
                if next_node not in closed_set:
                    anode = ANode(next_node, current_node.g_score + next_weight, self.get_heuristic_score(next_node))
                    for open_node in open_set:
                        if open_node.node.x == anode.node.x and open_node.node.y == anode.node.y:
                            anode_index = open_set.index(open_node)
                            if anode.get_f_score() < open_set[anode_index].get_f_score():
                                open_set.pop(anode_index)
                                open_set.insert(anode_index, anode)

                    else:
                        heapq.heappush(open_set, anode)
        print(1)




