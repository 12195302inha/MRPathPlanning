import heapq


class Node:
    def __lt__(self, other):
        return True


if __name__ == '__main__':
    a = list()
    heapq.heappush(a, (-1, Node()))
    heapq.heappush(a, (-1, Node()))
    print(heapq.heappop(a))
    print(heapq.heappop(a))