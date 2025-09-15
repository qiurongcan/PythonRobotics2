"""

D* grid planning

author: Nirnay Roy

See Wikipedia article (https://en.wikipedia.org/wiki/D*)
动态A*算法
Dynamic AStar
"""
import math
from sys import maxsize

import matplotlib.pyplot as plt

show_animation = True

# 状态类
class State:

    def __init__(self, x, y):
        self.x = x # x坐标
        self.y = y # y坐标
        self.parent = None # 父节点 用于回溯和重建路径
        self.state = "." # 状态 
        self.t = "new"  # tag for state [new 尚未处理 open 在开放列表待处理 close已经处理完成]
        self.h = 0 # 到目标的估计代价
        self.k = 0 # 帮助算法如何处理节点

    # 计算到另一个节点的代价
    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize # 返回无穷大

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    # 设置节点状态
    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state

# 地图类 地图中的每一个点需要用节点类进行维护
class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list
    # 获取指定节点的8邻域节点
    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    # 设置障碍物
    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1
        # 获取最小的k值
        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        if k_old == x.h:
            for y in self.map.get_neighbors(x):
                """
                1.如果是全新节点直接优化
                2.y是原本的父节点是x，但是代价发生了变化（因为此时的x已经最佳状态了）
                3.y的父节点不是x，但是通过x能够得到更优化的路径，则优化
                """
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(x, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    # 返回开放列表中 k值最小的节点
    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        """
        将一个节点以新的代价值(h_new)插入或重新插入到开放列表中
        """

        # 在添加h的时候，还需要维护历史最小的h值
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            # 历史最小值k是【它被关闭时的h值】
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        # 从开放节点列表中移除节点到关闭节点
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        # 如果障碍物在路径中
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        rx = []
        ry = []
        
        # 从目标节点开始反向搜索
        self.insert(end, 0.0)

        while True:
            # 反向搜索，从终点向起点搜索
            self.process_state()
            if start.t == "close": # 当起点被标记的时候，说明找到了最优路径
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        # 第一次搜索完成后 添加新的障碍物
        AddNewObstacle(self.map) # add new obstacle after the first search finished

        while tmp != end:
            tmp.set_state("*")
            rx.append(tmp.x)
            ry.append(tmp.y)
            if show_animation:
                plt.plot(rx, ry, "-r")
                plt.pause(0.01)
            
            # 调用A*算法重新规划的机制
            # 如果父节点是障碍物时， 只更新受影响的部分
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return rx, ry

    # 增量式更新，只规划遇到障碍物的部分
    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break

def AddNewObstacle(map:Map):
    ox, oy = [], []
    for i in range(5, 21):
        ox.append(i)
        oy.append(40)
    map.set_obstacle([(i, j) for i, j in zip(ox, oy)])
    if show_animation:
        plt.pause(0.001)
        plt.plot(ox, oy, ".g")

def main():
    m = Map(100, 100)
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10)
    for i in range(-10, 60):
        ox.append(60)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60)
    for i in range(-10, 61):
        ox.append(-10)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40)
        oy.append(60 - i)
    # 设置障碍物
    m.set_obstacle([(i, j) for i, j in zip(ox, oy)])

    start = [10, 10]
    goal = [50, 50]
    if show_animation:
        plt.plot(ox, oy, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")

    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]

    # ======================= 生成地图和地图中的起始点 ========================


    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()


if __name__ == '__main__':
    main()
