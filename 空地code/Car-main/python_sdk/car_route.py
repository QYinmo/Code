import copy
import numpy as np
from loguru import logger
from SC import calc_spline_course


class RoutePlanner:
    def __init__(self, enter_routes, leave_routes):
        # 路径格式为{1:([],[])....}前面x后面y
        self.enter_routes = enter_routes
        self.leave_routes = leave_routes

        self.MATCH = {1: "x", 2: "x", 3: "y", 4: "y", 5: "y", 6: "y",
                      7: "x", 8: "x", 9: "y", 10: "y", 11: "y", 12: "y"}

        # 区域划分
        self.REGION_BOUNDARIES = {
            'x_low': 1.9,
            'x_high': 3.4,
            'y_threshold': 2.2
        }

    def get_region(self, x: float, y: float) -> int:
        """获取坐标所在区域编号"""
        if y > self.REGION_BOUNDARIES['y_threshold']:
            if x < self.REGION_BOUNDARIES['x_low']:
                return 1
            elif x > self.REGION_BOUNDARIES['x_high']:
                return 5
            else:
                return 3
        else:
            if x < self.REGION_BOUNDARIES['x_low']:
                return 2
            elif x > self.REGION_BOUNDARIES['x_high']:
                return 6
            else:
                return 4

    def get_id(self, x: float, y: float) -> int:
        """获取精确区域ID"""
        region = self.get_region(x, y)

        if region == 1:
            return 1 if y > 3.05 else 2
        elif region == 2:
            return 3 if x < 1.0 else 4
        elif region == 3:
            return 5 if x < 2.7 else 6
        elif region == 4:
            return 7 if y > 1.55 else 8
        elif region == 5:
            return 9 if x < 4.1 else 10
        elif region == 6:
            return 11 if x < 4.1 else 12
        else:
            raise ValueError(f"Invalid coordinates ({x}, {y})")

    def validate_endpoint(self, x_list: list, y_list: list,
                          new_x: float, new_y: float,
                          min_dist: float = 0.1) -> bool:
        """
        验证新终点是否有效
        Args:
            x_list: 历史x坐标列表
            y_list: 历史y坐标列表
            new_x: 新x坐标
            new_y: 新y坐标
            min_dist: 最小移动距离阈值
        Returns:
            bool: 是否有效
        """
        # 改掉头阈值在cos_theta那里
        if len(x_list) != len(y_list):
            raise ValueError("x/y坐标列表长度不一致")
        if not x_list:
            return True
        last_x, last_y = x_list[-1], y_list[-1]
        move_dist = np.hypot(new_x - last_x, new_y - last_y)
        if move_dist < min_dist:
            logger.warning(f"移动距离不足: {move_dist:.3f}m < {min_dist}m")
            return False
        if len(x_list) >= 2:
            # 计算方向一致性
            hist_vec = np.array([last_x - x_list[-2], last_y - y_list[-2]])
            new_vec = np.array([new_x - last_x, new_y - last_y])
            if np.linalg.norm(hist_vec) > 1e-6 and np.linalg.norm(new_vec) > 1e-6:
                cos_theta = np.dot(
                    hist_vec, new_vec) / (np.linalg.norm(hist_vec) * np.linalg.norm(new_vec))
                if cos_theta < -0.3:  # 约107度以上
                    logger.warning(
                        f"方向突变: {np.degrees(np.arccos(cos_theta)):.1f}°")
                    return False
        return True

    def get_route(self, x: float, y: float,
                  x0: float, x0_fix: float,
                  y0: float, y0_fix: float,
                  dl: float = 0.1) -> tuple:
        """
        获取进入和离开路径
        Args:
            x,y: 当前位置
            x0,y0: 目标基准点
            x0_fix,y0_fix: 目标偏移量
            dl: 路径点间隔
        Returns:
            tuple: (enter_params, leave_params)
                   each params包含[cx, cy, cyaw, ck]
        """
        area_id = self.get_id(x0, y0)
        logger.info(f"规划区域ID: {area_id}")
        # 处理进入路径
        enter_x, enter_y = copy.deepcopy(self.enter_routes[area_id])
        match_axis = self.MATCH[area_id]
        # 添加修正终点
        if match_axis == "x":
            x_add = x0 + x0_fix * (1 if enter_x[-1] < enter_x[-2] else -1)
            y_add = enter_y[-1]
        else:
            y_add = y0 + y0_fix * (1 if enter_y[-1] < enter_y[-2] else -1)
            x_add = enter_x[-1]
        if self.validate_endpoint(enter_x, enter_y, x_add, y_add, 0.01):
            enter_x.append(x_add)
            enter_y.append(y_add)
        # 计算样条路径
        cx_enter, cy_enter, cyaw_enter, ck_enter, _ = calc_spline_course(
            enter_x, enter_y, ds=dl)
        # 处理离开路径
        leave_x, leave_y = copy.deepcopy(self.leave_routes[area_id])
        # 查找分割点
        split_idx = 0
        if match_axis == "x":
            while (split_idx < len(leave_x) - 1 and
                   ((x < leave_x[split_idx]) == (x < leave_x[0]))):
                split_idx += 1
        else:
            while (split_idx < len(leave_y) - 1 and
                   ((y < leave_y[split_idx]) == (y < leave_y[0]))):
                split_idx += 1
        if split_idx == len(leave_x if match_axis == "x" else leave_y) - 1:
            split_idx = 0
        # 计算离开路径样条
        cx_leave, cy_leave, cyaw_leave, ck_leave, _ = calc_spline_course(
            leave_x[split_idx:], leave_y[split_idx:], ds=dl)
        return (
            [cx_enter, cy_enter, cyaw_enter, ck_enter],
            [cx_leave, cy_leave, cyaw_leave, ck_leave]
        )

    def visualize_route(self, enter_params, leave_params):
        try:
            import matplotlib.pyplot as plt
            plt.figure(figsize=(10, 6))
            plt.plot(enter_params[0], enter_params[1],
                     'b-', label='Enter Path')
            plt.quiver(enter_params[0][::5], enter_params[1][::5],
                       np.cos(enter_params[2][::5]), np.sin(
                           enter_params[2][::5]),
                       color='blue', scale=15)
            plt.plot(leave_params[0], leave_params[1],
                     'r-', label='Leave Path')
            plt.quiver(leave_params[0][::5], leave_params[1][::5],
                       np.cos(leave_params[2][::5]), np.sin(
                           leave_params[2][::5]),
                       color='red', scale=15)
            plt.axis('equal')
            plt.legend()
            plt.grid()
            plt.title("Path Planning Result")
            plt.show()
        except ImportError:
            logger.warning("Matplotlib not installed, visualization skipped")
