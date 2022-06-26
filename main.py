import copy

import matplotlib.pyplot as plt

from rrt_modify import RRT
from a_star import AStar
import heapq
import random
import math


class Environment:
    def __init__(self, _width, _height, _obstacle_list, _robot_radius):
        self.width = _width
        self.height = _height
        self.obstacle_list = _obstacle_list
        self.robot_radius = _robot_radius


class SSSP:
    def __init__(self, _env):
        self.roadmaps = []
        self.colors = ["g", "c", "m", "y"]
        self.min_rand = 0
        self.max_rand = 30
        self.sampling_m = 2
        self.threshold_dis = 3
        self.env = _env
        self.start_node = []
        self.goal_node = []
        self.rrt = RRT(
            rand_area=[self.min_rand, self.max_rand],
            obstacle_list=self.env.obstacle_list,
            robot_radius=self.env.robot_radius,
            play_area=[0, 30, 0, 30]
        )
        self.a_star = AStar()

    def collide(self, prev_q, next_q, index):
        for j, q_state in enumerate(prev_q):
            if index == j:
                continue
            distance = math.sqrt((q_state.x - next_q[index].x) ** 2 + (q_state.y - next_q[index].y) ** 2)
            if distance < self.rrt.robot_radius * 2:
                return True
        return False

    def calculate_score(self, q): # bfs -> A* 알고리즘.
        cost_sum = 0
        for i, q_state in enumerate(q):
            dx = abs(q_state.x - self.goal_node[i].x)
            dy = abs(q_state.y - self.goal_node[i].y)
            cost_sum += math.sqrt(dx ** 2 + dy ** 2)
        return cost_sum

    @staticmethod
    def edge_connect(roadmap, q_state_new, max_weight):
        for node in roadmap:
            dx = abs(node.x - q_state_new.x)
            dy = abs(node.y - q_state_new.y)
            weight = math.sqrt(dx ** 2 + dy ** 2)
            if weight <= max_weight:
                q_state_new.edge.append((weight, node))
                node.edge.append((weight, q_state_new))

    def search(self):
        start_list = sssp.get_random_pos_list(2, 20, 20)
        goal_list = sssp.get_random_pos_list(2, 20, 20)
        for start, goal in zip(start_list, goal_list):
            self.rrt.set_position(start, goal)
            roadmap = self.rrt.planning(roadmap=True, animation=False)
            self.start_node.append(roadmap[0])
            self.goal_node.append(roadmap[-1])
            self.roadmaps.append(roadmap)

        while True:
            frontier = list()
            q_init = [roadmap[0] for roadmap in self.roadmaps]
            n = len(q_init)
            i_next = 0 # zero based
            parent = None
            # score와 로봇들의 현재 위치, next, parent
            heapq.heappush(frontier, (0, q_init, i_next, parent))  # score, Q, next, parent
            explored = list()
            explored.append((q_init, i_next))

            while frontier:
                s = heapq.heappop(frontier)

                # check all q is at goal
                s_q = s[1]
                for j, q in enumerate(s_q):
                    if q.x != self.goal_node[j].x or q.y != self.goal_node[j].y:
                        break
                else:
                    return s

                self.draw_graph(s_q)

                i = s[2]
                q_state_from = s_q[i]

                # vertex expansion via sampling
                for _ in range(self.sampling_m):
                    # 랜덤 노드 생성 후 stree로 새로운 new node 생성
                    q_state_rand = self.rrt.get_random_node()
                    weight, q_state_new = self.rrt.steer(q_state_from, q_state_rand, self.threshold_dis)

                    # 장애물에 걸리는지 확인 후 로드맵에 추가 및 edge 연결
                    if self.rrt.check_if_outside_play_area(q_state_new, self.rrt.play_area) and \
                            self.rrt.check_collision(q_state_new, self.rrt.obstacle_list, self.rrt.robot_radius):
                        self.roadmaps[i].append(q_state_new)
                        self.edge_connect(self.roadmaps[i], q_state_new, weight)
                        # self.draw_graph(q_state_rand)

                # search node expansion
                if i != n - 1:
                    i_prime = i + 1
                else:
                    i_prime = 0

                for weight, q_state_to in q_state_from.edge:
                    q_prime = copy.deepcopy(s_q) # q_prime 생성
                    q_prime[i] = q_state_to # 로봇을 이동
                    if (q_prime, i_prime) not in explored and not self.collide(s_q, q_prime, i):
                        score = self.calculate_score(q_prime) # bfs로 최소 거리 계산
                        heapq.heappush(frontier, (score, q_prime, i_prime, s)) # frontier에 push
                        explored.append((q_prime, i_prime)) # explored에 추가

    def get_random_pos_list(self, robot_n, max_x, max_y):
        pos_list = []
        while len(pos_list) < robot_n:
            pos = (random.randint(0, max_x), random.randint(0, max_y))
            if pos not in pos_list:
                for (ox, oy, size) in self.rrt.obstacle_list:
                    dx = ox - pos[0]
                    dy = oy - pos[1]
                    if dx * dx + dy * dy <= (size + self.rrt.robot_radius) ** 2:
                        break
                else:
                    pos_list.append(pos)
        return pos_list

    def draw_graph(self, s_q=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if s_q is not None:
            for q_state in s_q:
                plt.plot(q_state.x, q_state.y, "^k")
                if self.rrt.robot_radius > 0.0:
                    self.rrt.plot_circle(q_state.x, q_state.y, self.rrt.robot_radius, '-r')

        edge_x = []
        edge_y = []
        for i, roadmap in enumerate(self.roadmaps):
            for node in roadmap:
                for weight, next_node in node.edge:
                    if [next_node.x, node.x] not in edge_x or [next_node.y, node.y] not in edge_y:
                        plt.plot([node.x, next_node.x], [node.y, next_node.y], self.colors[i])
                        edge_x.append([node.x, next_node.x])
                        edge_y.append([node.y, next_node.y])
            plt.plot(self.start_node[i].x, self.start_node[i].y, "x" + self.colors[i])
            plt.plot(self.goal_node[i].x, self.goal_node[i].y, "o" + self.colors[i])

        for (ox, oy, size) in self.rrt.obstacle_list:
            self.rrt.plot_circle(ox, oy, size)

        if self.rrt.play_area is not None:
            plt.plot([self.rrt.play_area.xmin, self.rrt.play_area.xmax,
                      self.rrt.play_area.xmax, self.rrt.play_area.xmin,
                      self.rrt.play_area.xmin],
                     [self.rrt.play_area.ymin, self.rrt.play_area.ymin,
                      self.rrt.play_area.ymax, self.rrt.play_area.ymax,
                      self.rrt.play_area.ymin],
                     "-k")

        plt.axis("equal")
        plt.axis([-2, 30, -2, 30])
        plt.grid(True)
        plt.pause(0.01)


if __name__ == '__main__':
    obstacle_list = [(15, 15, 1), (13, 16, 2), (13, 18, 2), (13, 20, 2), (17, 15, 2),
                     (19, 15, 2), (18, 20, 1)]
    env = Environment(30, 30, obstacle_list, 0.8)
    sssp = SSSP(env)
    sssp.search()
