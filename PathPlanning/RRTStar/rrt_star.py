"""

Path planning Sample Code with RRT*

author: Atsushi Sakai(@Atsushi_twi)
改进之处：
- 为新节点选择代价最小的父节点 
    不是只依赖于“最近的节点”，而是选择一个从起点来的路径更短的父节点
- 重布线

RRT 是找到一条可行的路径
RRT* 是找到一条渐进趋于最优的路径 
"""

import math
import sys
import matplotlib.pyplot as plt
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from RRT.rrt import RRT

show_animation = True


class RRTStar(RRT):
    """
    Class for RRT Star planning
    """
    # 新增代价属性
    class Node(RRT.Node):
        def __init__(self, x, y):
            super().__init__(x, y)
            self.cost = 0.0

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=30.0,
                 path_resolution=1.0,
                 goal_sample_rate=20,
                 max_iter=300,
                 connect_circle_dist=50.0, #近邻搜索的连接半径圆参数
                 search_until_max_iter=False, # 
                 robot_radius=0.0):
        """
        Setting Parameter

        start:Start Position [x,y]
        goal:Goal Position [x,y]
        obstacleList:obstacle Positions [[x,y,size],...]
        randArea:Random Sampling Area [min,max]

        """
        super().__init__(start, goal, obstacle_list, rand_area, expand_dis,
                         path_resolution, goal_sample_rate, max_iter,
                         robot_radius=robot_radius)
        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])
        self.search_until_max_iter = search_until_max_iter
        self.node_list = []

    def planning(self, animation=True):
        """
        rrt star path planning

        animation: flag for animation on or off .
        """

        self.node_list = [self.start]
        for i in range(self.max_iter):
            print("Iter:", i, ", number of nodes:", len(self.node_list))
            # 1.选取一个随机点
            rnd = self.get_random_node()
            # 2.找到邻近点
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)
            # 3.扩展新节点
            new_node = self.steer(self.node_list[nearest_ind], rnd,
                                  self.expand_dis)
            near_node = self.node_list[nearest_ind]
            # 4.计算初始成本 路径长度
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x-near_node.x,
                           new_node.y-near_node.y)
            # 如果没有碰撞
            if self.check_collision(
                    new_node, self.obstacle_list, self.robot_radius):
                # 5.查找临近节点
                near_inds = self.find_near_nodes(new_node)
                # 6.寻找最优父节点
                node_with_updated_parent = self.choose_parent(
                    new_node, near_inds)
                # 如果找到最优父节点
                if node_with_updated_parent:
                    # 7.重布线
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)

            if animation:
                self.draw_graph(rnd)

            if ((not self.search_until_max_iter)
                    and new_node):  # if reaches goal
                last_index = self.search_best_goal_node()
                if last_index is not None:
                    return self.generate_final_course(last_index)

        print("reached max iteration")

        last_index = self.search_best_goal_node()
        if last_index is not None:
            # 生成最优路径
            return self.generate_final_course(last_index)

        return None

    def choose_parent(self, new_node, near_inds):
        """
        Computes the cheapest point to new_node contained in the list
        near_inds and set such a node as the parent of new_node.
            Arguments:
            --------
                new_node, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                near_inds: list
                    Indices of indices of the nodes what are near to new_node

            Returns.
            ------
                Node, a copy of new_node
        """
        # 查找是否存在近邻节点
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds: # 保留的是索引编号
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                # 计算代价
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)

        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        # 找到最小代价的索引，然后确定得到最小索引
        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        # 更新最小代价
        new_node.cost = min_cost

        return new_node

    def search_best_goal_node(self):
        # 计算所有节点到目标节点的距离
        dist_to_goal_list = [
            self.calc_dist_to_goal(n.x, n.y) for n in self.node_list
        ]
        # 筛选在扩展节点内的节点
        goal_inds = [
            dist_to_goal_list.index(i) for i in dist_to_goal_list
            if i <= self.expand_dis
        ]

        safe_goal_inds = []
        # 碰撞检测，筛选安全节点
        for goal_ind in goal_inds:
            t_node = self.steer(self.node_list[goal_ind], self.goal_node)
            if self.check_collision(
                    t_node, self.obstacle_list, self.robot_radius):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        # 计算各候选路径的总成本
        safe_goal_costs = [self.node_list[i].cost +
                           self.calc_dist_to_goal(self.node_list[i].x, self.node_list[i].y)
                           for i in safe_goal_inds]

        # 选择成本最小的最佳节点
        min_cost = min(safe_goal_costs)
        for i, cost in zip(safe_goal_inds, safe_goal_costs):
            if cost == min_cost:
                return i

        return None

    # 查找新节点半径范围内的节点
    def find_near_nodes(self, new_node):
        """
        1) defines a ball centered on new_node
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                new_node: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.node_list) + 1
        # 邻近半径公式
        r = self.connect_circle_dist * math.sqrt(math.log(nnode) / nnode)
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        # 确保邻近半径不超过单次扩展的最大距离
        if hasattr(self, 'expand_dis'):
            r = min(r, self.expand_dis)
        # 计算新节点到树中所有节点的距离平方
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list]
        # 筛选邻近节点索引
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    # 重布线
    def rewire(self, new_node, near_inds):
        """
            For each node in near_inds, this will check if it is cheaper to
            arrive to them from new_node.
            In such a case, this will re-assign the parent of the nodes in
            near_inds to new_node.
            Parameters:
            ----------
                new_node, Node
                    Node randomly added which can be joined to the tree

                near_inds, list of uints
                    A list of indices of the self.new_node which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in choose_parent.

        """
        for i in near_inds:
            near_node = self.node_list[i]
            # 注意：这里是新节点到近邻节点
            edge_node = self.steer(new_node, near_node)
            # 如果距离为0，则扩展失败
            if not edge_node:
                continue
            # 计算新节点到邻近节点的代价
            edge_node.cost = self.calc_new_cost(new_node, near_node)
            # 检查是否会碰撞
            no_collision = self.check_collision(
                edge_node, self.obstacle_list, self.robot_radius)
            # 比较代价是否更小了
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                # 更新所有以该节点为父节点的子节点
                for node in self.node_list:
                    if node.parent == self.node_list[i]:
                        node.parent = edge_node
                # 用新的edge_node替换原来的near_node
                self.node_list[i] = edge_node
                # 传播成本更新
                self.propagate_cost_to_leaves(self.node_list[i])

    # 递归计算代价
    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node):
        # 从这个节点开始，更新所有后代节点
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                # 当前节点又作为其他节点的父节点
                self.propagate_cost_to_leaves(node)


def main():
    print("Start " + __file__)

    # ====Search Path with RRT====
    obstacle_list = [
        (5, 5, 1),
        (3, 6, 2),
        (3, 8, 2),
        (3, 10, 2),
        (7, 5, 2),
        (9, 5, 2),
        (8, 10, 1),
        (6, 12, 1),
    ]  # [x,y,size(radius)]

    # Set Initial parameters
    rrt_star = RRTStar(
        start=[0, 0],
        goal=[6, 10],
        rand_area=[-2, 15],
        obstacle_list=obstacle_list,
        expand_dis=1,
        robot_radius=0.8)
    # 主函数
    path = rrt_star.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
            plt.grid(True)
            plt.show()


if __name__ == '__main__':
    main()
