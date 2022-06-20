import copy

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
        self.roadmap = []
        self.min_rand = 0
        self.max_rand = 30
        self.sampling_m = 5
        self.threshold_dis = 3
        self.rrt = RRT(
            rand_area=[0, 30],
            obstacle_list=_obstacle_list,
            robot_radius=_robot_radius,
            play_area=[0, 30, 0, 30]
        )

    def collide(self):
        return False

    def calculate_score(self):
        return 0

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
            # score와 로봇들의 현재 위치, next, parent
            heapq.heappush(frontier, (0, q_init, i_next, parent))  # score, Q, next, parent
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

                i = (s[2] - 1)  # zero based
                q_state_from = s_q[i]

                # vertex expansion via sampling
                for _ in range(self.sampling_m):
                    # 랜덤 노드 생성 후 stree로 새로운 new node 생성
                    q_state_rand = self.rrt.get_random_node()
                    q_state_new = self.rrt.steer(q_state_from, q_state_rand, self.threshold_dis)

                    # 장애물에 걸리는지 확인 후 로드맵에 추가 및 edge 연결
                    if self.rrt.check_if_outside_play_area(q_state_new, self.rrt.play_area) and \
                            self.rrt.check_collision(q_state_new, self.rrt.obstacle_list, self.rrt.robot_radius):
                        roadmaps[i].append(q_state_new)
                        q_state_new.edge.append(q_state_from)
                        q_state_from.edge.append(q_state_new)

                # search node expansion
                if i != n:
                    i_prime = i + 1
                else:
                    i_prime = 1

                for q_state_to in q_state_from.edge:
                    q_prime = copy.deepcopy(s_q)
                    q_prime[i] = q_state_to
                    if (q_prime, i_prime) not in explored and not self.collide():
                        score = self.calculate_score()
                        heapq.heappush(frontier, (score, q_prime, i_prime, s))
                        explored.append((q_prime, i_prime))

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


if __name__ == '__main__':
    obstacle_list = [(15, 15, 1), (13, 16, 2), (13, 18, 2), (13, 20, 2), (17, 15, 2),
                     (19, 15, 2), (18, 20, 1)]
    sssp = SSSP(obstacle_list, 0.8)
    start_list = sssp.get_random_pos_list(2, 20, 20)
    goal_list = sssp.get_random_pos_list(2, 20, 20)
    sssp.search(start_list, goal_list)
