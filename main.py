from rrt_modify import RRT
import heapq
import random
import math


class SSSP:

    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    def __init__(self, _obstacle_list, _robot_radius):
        self.obstacle_list = _obstacle_list
        self.robot_radius = _robot_radius
        self.roadmap = []
        self.min_rand = 0
        self.max_rand = 30
        self.sampling_m = 10
        self.threshold_dis = 3
        self.rrt = RRT(
            rand_area=[0, 30],
            obstacle_list=self.obstacle_list,
            robot_radius=self.robot_radius,
            play_area=[0, 30, 0, 30]
        )

    def get_random_node(self):
        rnd = self.Node(
            random.uniform(self.min_rand, self.max_rand),
            random.uniform(self.min_rand, self.max_rand))
        return rnd

    def steer(self, from_node, to_node, threshold_length=float("inf")):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        if threshold_length > d:
            return None

        new_node.x = d * math.cos(theta)
        new_node.y = d * math.sin(theta)

        new_node.parent = from_node

        return new_node

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta

    def search(self, start_list, goal_list):
        roadmaps = list()
        for start, goal in zip(start_list, goal_list):
            self.rrt.set_position(start, goal)
            roadmaps.append(self.rrt.planning(roadmap=True, animation=False))

        while True:
            frontier = list()
            q_init = [roadmap[0] for roadmap in roadmaps]
            n = len(q_init)
            i_next = 1
            parent = None
            heapq.heappush(frontier, (0, q_init, i_next, parent))# score, Q, next, parent
            explored = list()
            explored.append((q_init, i_next))

            while frontier:
                s = heapq.heappop(frontier)

                # check all q is at goal
                s_q = s[1]
                for j, q in enumerate(s_q):
                    if q != goal_list[j]:
                        break
                else:
                    return s

                i = (s[2] - 1) # zero based
                q_state_from = s_q[i]

                # vertex expansion via sampling
                for _ in range(self.sampling_m):
                    q_state_rand = self.get_random_node()
                    q_state_new = self.steer(q_state_from, q_state_rand, self.threshold_dis)
                    if q_state_new:
                        roadmaps[i].append(q_state_new)

                # search node expansion
                if i != n:
                    i_prime = i + 1
                else:
                    i_prime = 1

    def get_random_pos_list(self, robot_n, max_x, max_y):
        pos_list = []
        while len(pos_list) < robot_n:
            pos = (random.randint(0, max_x), random.randint(0, max_y))
            if pos not in pos_list:
                for (ox, oy, size) in self.obstacle_list:
                    dx = ox - pos[0]
                    dy = oy - pos[1]
                    if dx * dx + dy * dy <= (size + self.robot_radius) ** 2:
                        break
                else:
                    pos_list.append(pos)
        return pos_list


if __name__ == '__main__':
    obstacle_list = [(15, 15, 1), (13, 16, 2), (13, 18, 2), (13, 20, 2), (17, 15, 2),
                     (19, 15, 2), (18, 20, 1)]
    sssp = SSSP(obstacle_list, 0.8)
    start_list = sssp.get_random_pos_list(2, 20, 20)
    goal_list = sssp.get_random_pos_list(2, 20, 20)
    sssp.search(start_list, goal_list)