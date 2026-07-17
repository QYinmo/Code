import numpy as np
# 导入您的库
from FlightController.Solutions.PathPlanner import PFBPP, TrajectoryGenerator, AStar, ObstacleGenerator
from loguru import logger
import matplotlib.pyplot as plt
import sys

# ----------------------------------------------------------------------
# ！！！重要！！！
# 如果在您的环境中，FlightController.Solutions.PathPlanner 无法在独立脚本中直接运行，
# 请确保使用上次回复中的 Mock 类替换掉实际的导入语句。
# ----------------------------------------------------------------------

# 初始化您提供的 ObstacleGenerator
obstacle_gen = ObstacleGenerator(
    x_range1=550, x_range2=-500, y_range1=-1000, y_range2=100)

# 计算地图的 X 轴和 Y 轴边界
# 确保 min() 和 max() 顺序正确，以设置合理的轴限制
# 假设 ranges 属性是 obstacle_gen 的内部属性，包含构造函数传入的四个值
# 如果 obstacle_gen 没有 .ranges 属性，您需要手动定义这些值
# 示例中，我使用构造函数传入的字面值来计算范围
# 假设 x_range1 和 x_range2 是可访问的属性
x_ranges = [obstacle_gen.x_range1, obstacle_gen.x_range2]
# 假设 y_range1 和 y_range2 是可访问的属性
y_ranges = [obstacle_gen.y_range1, obstacle_gen.y_range2]

# 如果您的 ObstacleGenerator 实例没有直接暴露这些属性，请使用原始值:
MAP_X_RANGE = (min(550, -500), max(550, -500))  # (-500, 550)
MAP_Y_RANGE = (min(-1000, 100), max(-1000, 100))  # (-1000, 100)


def Astar_plan(obstacles, start, goal):
    """A*规划路径 (您的实际函数)"""
    start = np.array(start)
    goal = np.array(goal)
    start_tuple = tuple(start) if isinstance(start, np.ndarray) else start
    goal_tuple = tuple(goal) if isinstance(goal, np.ndarray) else goal

    planner = AStar(start_tuple, goal_tuple, "euclidean")
    planner.update_obs(obstacles)
    path, _ = planner.searching()

    path = path[::-1]

    logger.info(f"[MISSION] A* Path planning result: {path}")
    path = np.array(path)

    if path.size == 0:
        logger.error("[MISSION] A* Path planning failed")
        return None
    return path


def add_obs(x1, y1):
    """添加障碍物 (您的实际函数)"""
    obstacle_gen.add_obstacle(
        'circle', cx=x1, cy=y1, radius=85, filled=True)
    logger.info(
        f"Added circle obstacle at ({x1:.0f}, {y1:.0f}) with radius 85.")


# ==========================================================
# 交互式可视化实现
# ==========================================================

class PathPlannerTester:
    def __init__(self):
        self.start_point = None
        self.goal_point = None
        self.obstacles_to_draw = []

        # Matplotlib 配置
        self.fig, self.ax = plt.subplots(figsize=(10, 8))  # 稍微调整尺寸以适应长方形范围
        self.ax.set_title(
            "Interactive A* Path Planner (Obstacles: Left Click, S/G: Right Click)")

        # *** 修正部分：设置地图边界 ***
        self.ax.set_xlim(MAP_X_RANGE[0], MAP_X_RANGE[1])
        self.ax.set_ylim(MAP_Y_RANGE[0], MAP_Y_RANGE[1])

        self.ax.set_aspect('equal', adjustable='box')
        self.ax.grid(True, linestyle='--')

        # 绘图元素句柄
        self.path_line = None
        self.start_marker = None
        self.goal_marker = None

        self.config_step = 0  # 0: 放置起点, 1: 放置终点

        # 绑定鼠标点击事件
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)

        print("\n===============================================")
        print("         A* 路径规划交互式测试器")
        print("===============================================")
        print(f"地图范围: X: {MAP_X_RANGE}, Y: {MAP_Y_RANGE}")
        print("1. **左键点击 (Left Click)**: 放置障碍物 (半径 85)。")
        print("2. **右键点击 (Right Click)**: 放置或修改起点/终点。")
        print("   - 第一次右键点击: 放置起点 (红色)")
        print("   - 第二次右键点击: 放置终点 (绿色)")
        print("===============================================\n")

    def on_click(self, event):
        if event.xdata is None or event.ydata is None:
            return

        click_point = (event.xdata, event.ydata)

        # 左键点击：放置障碍物
        if event.button == 1:
            x1, y1 = click_point
            add_obs(x1, y1)  # 调用您的障碍物添加函数

            # 绘制障碍物
            circle = plt.Circle((x1, y1), 85, color='gray',
                                alpha=0.5, edgecolor='black', linewidth=1)
            self.ax.add_artist(circle)
            self.obstacles_to_draw.append(circle)

            # 如果起点和终点已设置，则重新规划路径
            if self.start_point and self.goal_point:
                self.calculate_and_draw_path()

        # 右键点击：放置起点/终点
        elif event.button == 3:
            if self.config_step == 0:
                self.start_point = click_point
                if self.start_marker:
                    self.start_marker.remove()
                self.start_marker = self.ax.plot(
                    click_point[0], click_point[1], 'rs', markersize=12, label='Start Point')[0]
                self.ax.legend()
                self.config_step = 1
                print(
                    f"Start Point set at: ({click_point[0]:.0f}, {click_point[1]:.0f}). Next: Place Goal.")
            elif self.config_step == 1:
                self.goal_point = click_point
                if self.goal_marker:
                    self.goal_marker.remove()
                self.goal_marker = self.ax.plot(
                    click_point[0], click_point[1], 'g*', markersize=18, label='Goal Point')[0]
                self.ax.legend()
                self.config_step = 0  # 循环回起点设置
                print(
                    f"Goal Point set at: ({click_point[0]:.0f}, {click_point[1]:.0f}). Calculating Path...")
                self.calculate_and_draw_path()

        self.fig.canvas.draw_idle()

    def calculate_and_draw_path(self):
        """调用您的 Astar_plan 并绘制结果"""
        if not self.start_point or not self.goal_point:
            return

        # 清除旧路径
        if self.path_line:
            self.path_line.remove()

        # 获取障碍物数据 (A* 需要的格式)
        obstacles_data = obstacle_gen.get_obstacles()

        # 调用您的 Astar_plan 函数
        path_segments = Astar_plan(
            obstacles_data, self.start_point, self.goal_point)

        if path_segments is not None and path_segments.size > 0:
            # 路径是一系列坐标点
            path_array = np.array(path_segments)
            # 绘制新路径 (红色实线)
            self.path_line = self.ax.plot(
                path_array[:, 0], path_array[:, 1], 'r-', linewidth=3, alpha=0.8, label='A* Path')[0]
            self.ax.legend()
            print(f"Path Found! Total steps: {len(path_segments)}")
        else:
            print("[WARNING] A* Path Failed to Find a Path! Check for obstacles.")

        self.fig.canvas.draw_idle()

    def run(self):
        """显示图形界面"""
        plt.show()


if __name__ == "__main__":
    # 运行交互式测试器
    tester = PathPlannerTester()
    tester.run()
