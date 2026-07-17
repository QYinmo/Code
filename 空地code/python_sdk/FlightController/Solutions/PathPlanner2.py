import time
from collections import deque
from typing import Any, List, Literal, Optional, Tuple, Union

import numpy as np
from loguru import logger
from matplotlib import pyplot as plt
import math
import heapq


class TrajectoryGenerator:
    """五次多项式轨迹生成器"""

    def __init__(
        self, start_pos, des_pos, T, start_vel=[0, 0, 0], des_vel=[0, 0, 0], start_acc=[0, 0, 0], des_acc=[0, 0, 0]
    ):
        """五次多项式轨迹生成器

        Args:
            start_pos (List[float,...]): 起始位置
            des_pos (List[float,...]): 目标位置
            T (float): 轨迹总时间
            start_vel (List[float,...], optional): 起始速度矢量
            des_vel (List[float,...], optional): 目标速度矢量
            start_acc (List[float,...], optional): 起始加速度矢量
            des_acc (List[float,...], optional): 目标加速度矢量
        """
        self.start_x = start_pos[0]
        self.start_y = start_pos[1]
        self.start_z = start_pos[2]

        self.des_x = des_pos[0]
        self.des_y = des_pos[1]
        self.des_z = des_pos[2]

        self.start_x_vel = start_vel[0]
        self.start_y_vel = start_vel[1]
        self.start_z_vel = start_vel[2]

        self.des_x_vel = des_vel[0]
        self.des_y_vel = des_vel[1]
        self.des_z_vel = des_vel[2]

        self.start_x_acc = start_acc[0]
        self.start_y_acc = start_acc[1]
        self.start_z_acc = start_acc[2]

        self.des_x_acc = des_acc[0]
        self.des_y_acc = des_acc[1]
        self.des_z_acc = des_acc[2]

        self.T = T

    def solve(self):
        A = np.array(
            [
                [0, 0, 0, 0, 0, 1],
                [self.T**5, self.T**4, self.T**3, self.T**2, self.T, 1],
                [0, 0, 0, 0, 1, 0],
                [5 * self.T**4, 4 * self.T**3, 3 * self.T**2, 2 * self.T, 1, 0],
                [0, 0, 0, 2, 0, 0],
                [20 * self.T**3, 12 * self.T**2, 6 * self.T, 2, 0, 0],
            ]
        )

        b_x = np.array(
            [[self.start_x], [self.des_x], [self.start_x_vel], [
                self.des_x_vel], [self.start_x_acc], [self.des_x_acc]]
        )

        b_y = np.array(
            [[self.start_y], [self.des_y], [self.start_y_vel], [
                self.des_y_vel], [self.start_y_acc], [self.des_y_acc]]
        )

        b_z = np.array(
            [[self.start_z], [self.des_z], [self.start_z_vel], [
                self.des_z_vel], [self.start_z_acc], [self.des_z_acc]]
        )

        self.x_c = np.linalg.solve(A, b_x)
        self.y_c = np.linalg.solve(A, b_y)
        self.z_c = np.linalg.solve(A, b_z)

    def calc_position(self, axis: Literal["x", "y", "z"], t: float) -> float:
        c = getattr(self, axis + "_c")
        return c[0] * t**5 + c[1] * t**4 + c[2] * t**3 + c[3] * t**2 + c[4] * t + c[5]

    def calc_position_xyz(self, t: float) -> Tuple[float, float, float]:
        return (
            self.calc_position("x", t),
            self.calc_position("y", t),
            self.calc_position("z", t),
        )

    def calc_velocity(self, axis: Literal["x", "y", "z"], t: float) -> float:
        c = getattr(self, axis + "_c")
        return 5 * c[0] * t**4 + 4 * c[1] * t**3 + 3 * c[2] * t**2 + 2 * c[3] * t + c[4]

    def calc_velocity_xyz(self, t: float) -> Tuple[float, float, float]:
        return (
            self.calc_velocity("x", t),
            self.calc_velocity("y", t),
            self.calc_velocity("z", t),
        )

    def calc_acceleration(self, axis: Literal["x", "y", "z"], t: float) -> float:
        c = getattr(self, axis + "_c")
        return 20 * c[0] * t**3 + 12 * c[1] * t**2 + 6 * c[2] * t + 2 * c[3]

    def calc_acceleration_xyz(self, t: float) -> Tuple[float, float, float]:
        return (
            self.calc_acceleration("x", t),
            self.calc_acceleration("y", t),
            self.calc_acceleration("z", t),
        )


class PFBPP(object):
    """
    Potential Field based path planner
    (基于势场的路径规划器)
    """

    # the number of previous positions used to check oscillations
    OSCILLATIONS_DETECTION_LENGTH = 4
    OSCILLATIONS_RETRY_FALLBACK = 10
    OSCILLATIONS_MAX_RETRY = 10

    def __init__(self) -> None:
        self._area_width = 30.0  # potential area width [m]
        self._grid_size = 0.5  # potential grid size [m]
        self._robot_radius = 1.0  # robot radius [m]
        self._atg = 5.0  # attractive potential
        self._rpg = 100.0  # repulsive potential
        self._attr_calced = False
        self._repu_calced = False

    def _calc_potential_field(self):
        """计算势场"""
        minx = min(min(self._ox), self._sx, self._gx) - self._area_width / 2.0
        miny = min(min(self._oy), self._sy, self._gy) - self._area_width / 2.0
        maxx = max(max(self._ox), self._sx, self._gx) + self._area_width / 2.0
        maxy = max(max(self._oy), self._sy, self._gy) + self._area_width / 2.0
        xw = int(round((maxx - minx) / self._grid_size))
        yw = int(round((maxy - miny) / self._grid_size))

        # calc each potential
        pmap = [[0.0 for i in range(yw)] for i in range(xw)]

        if not self._attr_calced:
            self._ugxy = []
        if not self._repu_calced:
            self._uoxy = []
        i = 0
        for ix in range(xw):
            x = ix * self._grid_size + minx
            for iy in range(yw):
                y = iy * self._grid_size + miny
                if not self._attr_calced:
                    self._ugxy.append(self._calc_attractive_potential(x, y))
                if not self._repu_calced:
                    self._uoxy.append(self._calc_repulsive_potential(x, y))
                uf = self._ugxy[i] + self._uoxy[i]
                pmap[ix][iy] = uf
                i += 1
        self._attr_calced = self._repu_calced = True
        return pmap, minx, miny

    def _calc_attractive_potential(self, x, y):
        """计算吸引势"""
        return 0.5 * self._atg * np.hypot(x - self._gx, y - self._gy)

    def _calc_repulsive_potential(self, x, y):
        """计算斥力势"""
        # search nearest obstacle
        minid = -1
        dmin = float("inf")
        for i, _ in enumerate(self._ox):
            d = np.hypot(x - self._ox[i], y - self._oy[i])
            if dmin >= d:
                dmin = d
                minid = i

        # calc repulsive potential
        dq = np.hypot(x - self._ox[minid], y - self._oy[minid])

        if dq <= self._robot_radius:
            if dq <= 0.1:
                dq = 0.1

            return 0.5 * self._rpg * (1.0 / dq - 1.0 / self._robot_radius) ** 2
        else:
            return 0.0

    def _oscillations_detection(self, previous_ids, ix, iy):
        """振荡检测器"""
        previous_ids.append((ix, iy))

        if len(previous_ids) > PFBPP.OSCILLATIONS_DETECTION_LENGTH:
            previous_ids.popleft()

        # check if contains any duplicates by copying into a set
        previous_ids_set = set()
        for index in previous_ids:
            if index in previous_ids_set:
                return True
            else:
                previous_ids_set.add(index)
        return False

    def set_plan_path(self, start_point, goal_point):
        """设置规划起点和终点

        Args:
            start_point (二维点): 起点 / m
            goal_point (二维点): 终点 / m
        """
        self._start_point = start_point
        self._goal_point = goal_point
        self._sx, self._sy = self._start_point
        self._gx, self._gy = self._goal_point
        self._attr_calced = False

    def set_obstacle(self, obstacle_list):
        """设置障碍物

        Args:
            obstacle_list (二维点集): 障碍物列表 / m
        """
        self._obstacle_list = obstacle_list
        self._ox = []
        self._oy = []
        for point in self._obstacle_list:
            self._ox.append(point[0])
            self._oy.append(point[1])
        self._repu_calced = False

    def create_boundry(self, p1, p2, space=None) -> list:
        """创建边界

        Args:
            p1 (tuple): 点1 / m
            p2 (tuple): 点2 / m
            space (float): 点间距 / m 默认取网格间距

        Returns:
            list: 边界点列表 / m
        """
        obstacle = []
        s = space if space else self._grid_size
        s = abs(s)
        for x in np.arange(p1[0], p2[0], s if p1[0] < p2[0] else -s):
            obstacle.append((x, p1[1]))
            obstacle.append((x, p2[1]))
        for y in np.arange(p1[1], p2[1], s if p1[1] < p2[1] else -s):
            obstacle.append((p1[0], y))
            obstacle.append((p2[0], y))
        return obstacle

    def set_params(self, area_width, grid_size, robot_radius):
        """设置参数

        Args:
            area_width (float): 地图宽度 / m
            grid_size (float): 网格大小 / m
            robot_radius (float): 机器人半径 / m
        """
        self._area_width = area_width
        self._grid_size = grid_size
        self._robot_radius = robot_radius
        self._repu_calced = self._attr_calced = False

    def set_attractive_gain(self, attractive_gain=5.0):
        """设置吸引势增益

        Args:
            attractive_gain (float): 吸引势增益
        """
        self._atg = attractive_gain
        self._attr_calced = False

    def set_repulsive_gain(self, repulsive_gain=100.0):
        """设置斥力势增益

        Args:
            repulsive_gain (float): 斥力势增益
        """
        self._rpg = repulsive_gain
        self._repu_calced = False

    def calc_potential_field(self):
        """计算势场"""
        # calc potential field
        self._pmap, self._minx, self._miny = self._calc_potential_field()
        logger.info("Potential field calculation finished.")

    def run_planner(self, debug=False, osc_retry=True):
        """运行规划器

        Args:
            debug (bool): 是否显示调试图像
            osc_retry (bool): 检测到振荡是否重试

        Returns:
            path (list): 路径点列表 / m, None表示规划失败
        """
        if not (self._attr_calced and self._repu_calced):
            self.calc_potential_field()

        # search path
        d = np.hypot(self._sx - self._gx, self._sy - self._gy)
        ix = round((self._sx - self._minx) / self._grid_size)
        iy = round((self._sy - self._miny) / self._grid_size)
        gix = round((self._gx - self._minx) / self._grid_size)
        giy = round((self._gy - self._miny) / self._grid_size)

        if debug:
            plt.clf()
            plt.grid(True)
            plt.axis("equal")
            data = np.array(self._pmap).T
            plt.pcolor(data, vmax=100.0, cmap=plt.cm.Blues)  # type: ignore
            plt.plot(ix, iy, "*k")
            plt.plot(gix, giy, "*m")

        points = [(self._sx, self._sy)]

        motion = [[1, 0], [0, 1], [-1, 0], [0, -1],
                  [-1, -1], [-1, 1], [1, -1], [1, 1]]
        previous_ids = deque()

        while d >= self._grid_size:
            minp = float("inf")
            minix, miniy = -1, -1
            for i, _ in enumerate(motion):
                inx = int(ix + motion[i][0])
                iny = int(iy + motion[i][1])
                if inx >= len(self._pmap) or iny >= len(self._pmap[0]) or inx < 0 or iny < 0:
                    p = float("inf")  # outside area
                    logger.error("Outside area! ({},{})".format(inx, iny))
                else:
                    p = self._pmap[inx][iny]
                if minp > p:
                    minp = p
                    minix = inx
                    miniy = iny
            ix = minix
            iy = miniy
            xp = ix * self._grid_size + self._minx
            yp = iy * self._grid_size + self._miny
            d = np.hypot(self._gx - xp, self._gy - yp)
            points.append((xp, yp))
            if self._oscillations_detection(previous_ids, ix, iy):
                logger.warning(
                    "Oscillation detected at ({},{})!".format(ix, iy))
                if not osc_retry:
                    return None
                old_radius = self._robot_radius
                points = points[: -PFBPP.OSCILLATIONS_RETRY_FALLBACK]
                old_start_point = self._start_point
                if PFBPP.OSCILLATIONS_RETRY_FALLBACK > PFBPP.OSCILLATIONS_DETECTION_LENGTH:
                    self._start_point = points[-1] if len(
                        points) > 0 else old_start_point
                    self._sx, self._sy = self._start_point
                    self._attr_calced = False
                    logger.info(f"Oscillation fallback to {self._start_point}")
                new_point = None
                retry = 0
                while new_point is None:
                    self._robot_radius += self._grid_size * retry
                    self._repu_calced = False
                    retry += 1
                    if retry > PFBPP.OSCILLATIONS_MAX_RETRY:
                        logger.error("Oscillation retry reached max retry!")
                        return None
                    logger.debug(
                        f"Retry#{retry} with radius {self._robot_radius}")
                    new_point = self.run_planner(debug=debug, osc_retry=False)
                self._robot_radius = old_radius
                self._repu_calced = False
                if PFBPP.OSCILLATIONS_RETRY_FALLBACK > PFBPP.OSCILLATIONS_DETECTION_LENGTH:
                    self._start_point = old_start_point
                    self._sx, self._sy = self._start_point
                    self._attr_calced = False
                    return points + new_point
                else:
                    return new_point
            if debug:
                plt.plot(ix, iy, ".r")
                plt.pause(0.01)
        logger.info(f"Plan success")
        points.append(self._goal_point)
        return points


class ObstacleGenerator:
    def __init__(self, x_range1=-500, x_range2=500, y_range1=-500, y_range2=500):
        """初始化障碍物生成器

        Args:
            x_range (int): x轴范围
            y_range (int): y轴范围
        """
        self.x_range1 = x_range1
        self.x_range2 = x_range2
        self.y_range1 = y_range1
        self.y_range2 = y_range2
        self.obstacles = set()  # 使用集合存储障碍物

        # 预定义形状模板
        self.shape_templates = {
            'wall': self._generate_wall,
            'rectangle': self._generate_rectangle,
            'line': self._generate_line,
            'circle': self._generate_circle
        }

    def add_obstacle(self, shape_type, **kwargs):
        """添加障碍物（始终返回集合）

        Args:
            shape_type (str): 形状类型 ('wall', 'rectangle', 'line', 'circle')
            **kwargs: 形状特定参数
        Returns:
            set: 新增的障碍物坐标集合
        """
        generator = self.shape_templates.get(shape_type)
        if generator:
            points = generator(**kwargs)
            self.obstacles.update(points)
            return points  # 返回新增的点集
        else:
            raise ValueError(f"Unsupported shape type: {shape_type}")

    def clear_obstacles(self):
        """清除所有障碍物"""
        self.obstacles.clear()

    def get_obstacles(self):
        """获取障碍物集合（始终返回集合的副本）

        Returns:
            set: 障碍物坐标集合的副本
        """
        return self.obstacles.copy()

    # 以下是形状生成方法（每个方法都返回集合）
    def _generate_wall(self, thickness=1):
        """生成边界墙（返回集合）"""
        points = set()
        # 上下墙
        x = self.x_range1
        for x in range(self.x_range1, self.x_range2):
            for t in range(thickness):
                points.add((x, self.y_range1+t))
                points.add((x, self.y_range2 - t))

        # 左右墙
        for y in range(self.y_range1, self.y_range2):
            for t in range(thickness):
                points.add((self.x_range1+t, y))
                points.add((self.x_range2 - t, y))
        return points

    def _generate_rectangle(self, x1, y1, x2, y2, filled=False):
        """生成矩形障碍物（返回集合）"""
        points = set()
        x1, x2 = sorted([x1, x2])
        y1, y2 = sorted([y1, y2])

        # 边界检查
        x1 = max(self.x_range1, min(x1, self.x_range2-1))
        x2 = max(self.x_range1, min(x2, self.x_range2-1))
        y1 = max(self.y_range1, min(y1, self.y_range2-1))
        y2 = max(self.y_range1, min(y2, self.y_range2-1))

        # 边框
        for x in range(x1, x2+1):
            points.add((x, y1))
            points.add((x, y2))
        for y in range(y1, y2+1):
            points.add((x1, y))
            points.add((x2, y))

        # 填充
        if filled:
            for x in range(x1+1, x2):
                for y in range(y1+1, y2):
                    points.add((x, y))
        return points

    def _generate_line(self, x1, y1, x2, y2, thickness=1):
        """生成线段障碍物（返回集合）"""
        points = set()
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        while True:
            # 中心点
            points.add((x1, y1))

            # 线宽
            for t in range(1, thickness):
                if dx > dy:
                    points.add((x1, y1 + t))
                    points.add((x1, y1 - t))
                else:
                    points.add((x1 + t, y1))
                    points.add((x1 - t, y1))

            if x1 == x2 and y1 == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x1 += sx
            if e2 < dx:
                err += dx
                y1 += sy
        return points

    def _generate_circle(self, cx, cy, radius, filled=False):
        """生成圆形障碍物（返回集合）"""
        points = set()
        for x in range(max(self.x_range1, cx - radius), min(self.x_range2, cx + radius + 1)):
            for y in range(max(self.y_range1, cy - radius), min(self.y_range2, cy + radius + 1)):
                dx = x - cx
                dy = y - cy
                distance_sq = dx*dx + dy*dy

                if filled:
                    if distance_sq <= radius*radius:
                        points.add((x, y))
                else:
                    if abs(distance_sq - radius*radius) <= radius:
                        points.add((x, y))
        return points


class AStar:
    """AStar 将代价（g）+ 启发式值（h）作为优先级"""

    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start  # 起始状态/节点
        self.s_goal = s_goal    # 目标状态/节点
        self.heuristic_type = heuristic_type  # 启发式函数类型（例如“manhattan”或“euclidean”）
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]

        self.u_set = self.motions  # 可行的移动集合
        self.obs = set()  # 障碍物的位置集合 #障碍物初始化，env即地图

        self.OPEN = []  # 优先队列 / OPEN 列表 #open list...实际列表！
        self.CLOSED = []  # CLOSED 列表 / 已访问顺序
        self.PARENT = dict()  # 记录父节点 #父节点...实际字典！
        self.g = dict()  # 到达节点的代价 #点对应的实际到起点的最短距离

    def update_obs(self, obstacle_list):
        self.obs = obstacle_list

    def searching(self):
        """
        A* 搜索算法。
        :return: 路径，已访问节点的顺序
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        # 将起始节点加入优先队列，优先级为 f_value(s_start)
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:  # 当优先队列不为空时
            _, s = heapq.heappop(self.OPEN)  # 弹出 f_value 最小的节点
            self.CLOSED.append(s)  # 将当前节点加入已关闭列表

            if s == self.s_goal:  # 停止条件：如果当前节点是目标节点
                break

            for s_n in self.get_neighbor(s):  # 遍历当前节点 s 的所有邻居 s_n
                new_cost = self.g[s] + self.cost(s, s_n)  # 计算从起点到邻居 s_n 的新代价

                if s_n not in self.g:  # 如果邻居 s_n 尚未被访问过，将其代价初始化为无穷大
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # 更新代价的条件：如果新代价更小
                    self.g[s_n] = new_cost  # 更新到达 s_n 的代价
                    self.PARENT[s_n] = s   # 更新 s_n 的父节点为 s
                    # 将 s_n 加入优先队列，优先级为 f_value(s_n)
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED  # 返回规划出的路径和已访问节点顺序

    def searching_repeated_astar(self, e):
        """
        重复 A* 搜索。
        :param e: A* 算法的启发式权重
        :return: 路径和已访问节点的顺序
        """

        path, visited = [], []

        while e >= 1:  # 当权重 e 大于等于 1 时循环
            p_k, v_k = self.repeated_searching(e)  # 执行一次带权重的 A* 搜索
            path.append(p_k)    # 将本次搜索到的路径添加到路径列表中
            visited.append(v_k)  # 将本次搜索访问的节点添加到已访问节点列表中
            e -= 0.5            # 减小权重 e

        return path, visited

    def repeated_searching(self, e):
        """
        运行带权重 e 的 A* 搜索。
        :param s_start: 起始状态
        :param s_goal: 目标状态
        :param e: A* 的权重
        :return: 路径和已访问节点的顺序。
        """

        g = {self.s_start: 0, self.s_goal: float("inf")}  # g 值（从起点到当前节点的代价）字典
        PARENT = {self.s_start: self.s_start}            # 父节点字典
        OPEN = []                              # 优先队列
        CLOSED = []                            # 已访问节点列表
        # 将起始节点加入优先队列，优先级为 g[self.s_start] + e * heuristic(self.s_start)
        heapq.heappush(OPEN,
                       (g[self.s_start] + e * self.heuristic(self.s_start), self.s_start))

        while OPEN:  # 当优先队列不为空时
            _, s = heapq.heappop(OPEN)  # 弹出 f_value 最小的节点
            CLOSED.append(s)           # 将当前节点加入已关闭列表

            if s == self.s_goal:  # 如果当前节点是目标节点
                break

            for s_n in self.get_neighbor(s):  # 遍历当前节点 s 的所有邻居 s_n
                new_cost = g[s] + self.cost(s, s_n)  # 计算从起点到邻居 s_n 的新代价

                if s_n not in g:  # 如果邻居 s_n 尚未被访问过，将其代价初始化为无穷大
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # 更新代价的条件：如果新代价更小
                    g[s_n] = new_cost  # 更新到达 s_n 的代价
                    PARENT[s_n] = s    # 更新 s_n 的父节点为 s
                    # 将 s_n 加入优先队列，优先级为 g[s_n] + e * heuristic(s_n)
                    heapq.heappush(
                        OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED  # 返回规划出的路径和已访问节点顺序

    def get_neighbor(self, s):
        """
        查找状态 s 的邻居，这些邻居不能是障碍物。
        :param s: 状态/节点
        :return: 邻居列表
        """

        # 通过遍历 Env 中定义的移动集合 u_set，计算邻居的坐标
        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        计算此移动的代价。
        :param s_start: 起始节点
        :param s_goal: 结束节点
        :return: 此移动的代价
        :note: 代价函数可以更复杂！
        """

        if self.is_collision(s_start, s_goal):  # 如果起始节点到目标节点之间发生碰撞
            return math.inf  # 返回无穷大的代价

        # 如果没有碰撞，则代价为两点之间的欧几里得距离
        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        检查线段 (s_start, s_end) 是否发生碰撞。
        :param s_start: 起始节点
        :param s_end: 结束节点
        :return: True: 发生碰撞 / False: 未发生碰撞
        """

        # 如果起始点或结束点本身就是障碍物，则视为碰撞
        if s_start in self.obs or s_end in self.obs:
            return True

        # 检查对角线移动时的中间点是否为障碍物
        # 仅当 X 和 Y 坐标都发生变化（即对角线移动）时才进行此检查
        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            # 判断对角线的方向，并确定两个中间点 s1 和 s2
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                # 如果是这种对角线方向 (例如从左下到右上)
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                # 如果是另一种对角线方向 (例如从左上到右下)
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            # 如果这两个中间点中的任何一个在障碍物中，则视为碰撞
            if s1 in self.obs or s2 in self.obs:
                return True

        return False  # 未发生碰撞

    def f_value(self, s):
        """
        计算 f 值：f = g + h。（g: 到达代价, h: 启发式值）
        :param s: 当前状态/节点
        :return: f 值
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        根据 PARENT 字典提取路径。
        :return: 规划出的路径
        """

        path = [self.s_goal]  # 从目标节点开始构建路径
        s = self.s_goal

        while True:
            s = PARENT[s]  # 回溯到父节点
            path.append(s)  # 将父节点添加到路径中

            if s == self.s_start:  # 如果回溯到起始节点，则停止
                break

        return list(path)  # 返回路径列表

    def heuristic(self, s):
        """
        计算启发式值。
        :param s: 当前节点 (状态)
        :return: 启发式函数值
        """

        heuristic_type = self.heuristic_type  # 启发式函数类型
        goal = self.s_goal                    # 目标节点

        if heuristic_type == "manhattan":
            # 曼哈顿距离（L1 范数）
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            # 欧几里得距离（L2 范数）
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

    def run_Astar(self, s_start, s_goal):
        """
        运行 A* 算法的主函数。
        :return: None
        """
        self.s_start = s_start
        self.s_goal = s_goal

        # 初始化 A* 规划器，使用欧几里得距离作为启发式函数
        astar = AStar(self.s_start, self.s_goal, "euclidean")

        # 执行 A* 搜索
        path, visited = astar.repeated_searching(1.2)

        # 返回规划出的路径和已访问节点的顺序
        return path
