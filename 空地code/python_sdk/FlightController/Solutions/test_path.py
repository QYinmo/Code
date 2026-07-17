import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from matplotlib.patches import Circle
from PathPlanner import *
import matplotlib.font_manager as fm


class PathPlannerTest:
    def __init__(self):
        # 设置中文字体（关键修改）
        plt.rcParams['font.sans-serif'] = ['SimHei']  # Windows系统
        plt.rcParams['axes.unicode_minus'] = False  # 解决负号显示问题

        # 或者指定具体字体路径（跨平台方案）
        # font_path = '/usr/share/fonts/windows/SimHei.ttf'  # Linux路径示例
        # font_prop = fm.FontProperties(fname=font_path)
        # plt.rcParams['font.family'] = font_prop.get_name()

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        plt.subplots_adjust(bottom=0.2)
        self.ax.set_title("路径规划测试 - 左键设置障碍物，右键设置起点/终点")
        self.ax.grid(True)
        self.ax.set_xlim(0, 30)
        self.ax.set_ylim(0, 30)

        # 初始化规划器
        self.planner = PFBPP()
        self.planner.set_params(area_width=30, grid_size=0.5, robot_radius=1.5)

        # 存储交互数据
        self.obstacles = []
        self.start_point = None
        self.goal_point = None
        self.click_mode = "obstacle"  # obstacle/start/goal

        # 创建中文按钮
        self._create_buttons()

        # 绑定鼠标事件
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)

        # 可视化元素
        self.obstacle_plot = self.ax.plot(
            [], [], 'ro', markersize=5, label='障碍物')[0]
        self.start_marker = self.ax.plot(
            [], [], 'gs', markersize=10, markeredgewidth=2, label='起点')[0]
        self.goal_marker = self.ax.plot(
            [], [], 'y*', markersize=15, markeredgewidth=2, label='终点')[0]
        self.path_line = self.ax.plot(
            [], [], 'b-', linewidth=2, label='规划路径')[0]
        self.robot_radius_viz = None
        self.ax.legend(loc='upper right')

        plt.show()

    def _create_buttons(self):
        """创建中文控制按钮"""
        ax_obstacle = plt.axes([0.2, 0.05, 0.15, 0.05])
        ax_start = plt.axes([0.4, 0.05, 0.15, 0.05])
        ax_goal = plt.axes([0.6, 0.05, 0.15, 0.05])
        ax_run = plt.axes([0.8, 0.05, 0.15, 0.05])

        self.btn_obstacle = Button(ax_obstacle, '障碍物模式')
        self.btn_start = Button(ax_start, '设置起点')
        self.btn_goal = Button(ax_goal, '设置终点')
        self.btn_run = Button(ax_run, '开始规划')

        self.btn_obstacle.on_clicked(lambda _: self._set_mode("obstacle"))
        self.btn_start.on_clicked(lambda _: self._set_mode("start"))
        self.btn_goal.on_clicked(lambda _: self._set_mode("goal"))
        self.btn_run.on_clicked(lambda _: self._run_planner())

    def _set_mode(self, mode):
        """设置点击模式（中文提示）"""
        self.click_mode = mode
        status_text = {
            "obstacle": "障碍物模式 - 左键添加障碍物",
            "start": "起点模式 - 左键设置起点位置",
            "goal": "终点模式 - 左键设置终点位置"
        }
        self.ax.set_title(status_text[mode])
        self.fig.canvas.draw()

    def _on_click(self, event):
        """处理鼠标点击事件（添加中文日志）"""
        if event.inaxes != self.ax:
            return

        x, y = event.xdata, event.ydata

        if self.click_mode == "obstacle" and event.button == 1:
            self.obstacles.append((x, y))
            print(f"添加障碍物坐标: ({x:.2f}, {y:.2f})")
            self._update_plot()

        elif self.click_mode == "start" and event.button == 1:
            self.start_point = (x, y)
            print(f"设置起点坐标: ({x:.2f}, {y:.2f})")
            self._update_plot()

        elif self.click_mode == "goal" and event.button == 1:
            self.goal_point = (x, y)
            print(f"设置终点坐标: ({x:.2f}, {y:.2f})")
            self._update_plot()

    def _update_plot(self):
        """更新可视化（添加中文标签）"""
        # 更新障碍物
        if self.obstacles:
            ox, oy = zip(*self.obstacles)
            self.obstacle_plot.set_data(ox, oy)

        # 更新起点/终点
        if self.start_point:
            self.start_marker.set_data(
                [self.start_point[0]], [self.start_point[1]])

        if self.goal_point:
            self.goal_marker.set_data(
                [self.goal_point[0]], [self.goal_point[1]])

        # 更新机器人半径显示
        if self.robot_radius_viz:
            self.robot_radius_viz.remove()

        if self.start_point:
            self.robot_radius_viz = Circle(
                self.start_point, self.planner._robot_radius,
                fill=False, color='g', linestyle='--', linewidth=1
            )
            self.ax.add_patch(self.robot_radius_viz)
            self.robot_radius_viz.set_label('安全半径')

        # 更新图例
        handles, labels = self.ax.get_legend_handles_labels()
        if self.robot_radius_viz and '安全半径' not in labels:
            handles.append(self.robot_radius_viz)
        self.ax.legend(handles=handles)

        self.fig.canvas.draw()

    def _run_planner(self):
        """执行路径规划（中文输出）"""
        if not self.start_point or not self.goal_point:
            self.ax.set_title("规划失败：请先设置起点和终点！", color='red')
            self.fig.canvas.draw()
            return

        print("\n===== 开始路径规划 =====")
        print(f"起点: {self.start_point}")
        print(f"终点: {self.goal_point}")
        print(f"障碍物数量: {len(self.obstacles)}")

        # 设置规划参数
        self.planner.set_plan_path(self.start_point, self.goal_point)
        self.planner.set_obstacle(self.obstacles)

        # 计算势场
        print("计算势场中...")
        self.planner.calc_potential_field()

        # 可视化势场
        self._plot_potential_field()

        # 运行规划器
        print("正在规划路径...")
        path = self.planner.run_planner(debug=True)

        # 显示路径
        if path:
            px, py = zip(*path)
            self.path_line.set_data(px, py)
            length = self._calc_path_length(path)
            title = f"规划成功！路径长度: {length:.2f} 米"
            self.ax.set_title(title, color='green')
            print(title)
        else:
            self.ax.set_title("规划失败：无法找到可行路径！", color='red')
            print("规划失败：可能被障碍物包围或参数设置不当")

        self.fig.canvas.draw()

    def _plot_potential_field(self):
        """绘制势场热力图（中文标签）"""
        # 清除旧的热力图
        for coll in self.ax.collections:
            if isinstance(coll, plt.cm.ScalarMappable):
                coll.remove()

        # 创建网格坐标
        x = np.linspace(self.planner._minx,
                        self.planner._minx +
                        len(self.planner._pmap)*self.planner._grid_size,
                        len(self.planner._pmap))
        y = np.linspace(self.planner._miny,
                        self.planner._miny +
                        len(self.planner._pmap[0])*self.planner._grid_size,
                        len(self.planner._pmap[0]))

        # 绘制势场
        potential = np.array(self.planner._pmap).T
        im = self.ax.pcolormesh(x, y, potential,
                                shading='auto', cmap='jet',
                                alpha=0.5, vmin=0, vmax=100)

        # 添加中文颜色条
        cbar = plt.colorbar(im, ax=self.ax)
        cbar.set_label('势场强度', rotation=270, labelpad=15)

    def _calc_path_length(self, path):
        """计算路径长度"""
        length = 0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            length += np.hypot(dx, dy)
        return length


if __name__ == "__main__":
    # 启动测试界面
    print("=== 路径规划测试程序 ===")
    print("说明：")
    print("1. 使用按钮切换模式")
    print("2. 左键添加障碍物/设置起点终点")
    print("3. 点击'开始规划'运行算法\n")

    test_app = PathPlannerTest()
