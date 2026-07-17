import numpy as np
import time
from loguru import logger
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from SC import Controller, State
from car_route import RoutePlanner


class DroneNavigationController:
    def __init__(self, *args, **kwargs):
        self.fc: FC_Like = kwargs["fc"]
        self.radar: LD_Radar = kwargs["radar"]
        self.enter_routes = {}
        self.leave_routes = {}
        self.rp = RoutePlanner(self.enter_routes, self.leave_routes)
        # 导航参数
        self.base_x = 0
        self.base_y = 0
        self.speed = 0.0
        self.target_speed = 200
        self.dl = 0.03
        self.DT = 0.1
        # 舵机参数配置
        self._init_servo_params()

    def _init_servo_params(self):
        """初始化舵机控制参数"""
        self.pulse_list = [1000, 1100, 1200, 1300, 1400, 1500,
                           1600, 1700, 1800, 1900, 2000, 2100, 2200, 2300]
        self.deg_list = [35, 31, 24, 16, 7, 0, -
                         6, -9, -13, -19, -20.5, -22, -23.7, -24]
        self.rad_list = np.deg2rad(self.deg_list)
        self.max_angle_r = self.deg_list[0]
        self.max_angle_l = self.deg_list[-1]
        self.max_angle_r_rad = self.rad_list[0]
        self.max_angle_l_rad = self.rad_list[-1]
        # 车辆参数
        self.Axle_spacing = 0.146  # 前后轴距(m)
        self.Wheel_spacing = 0.163  # 轮距(m)
        self.WHEEL_PERIMETER = 0.21049  # 轮周长(m)

    def calibrate(self):
        """校准初始位置"""
        x, y, _ = self.get_xy_yaw()
        self.base_x = x - 1.25
        self.base_y = y - 0.38
        logger.info(f"校准完成 - 基准坐标: ({self.base_x:.2f}, {self.base_y:.2f})")

    def get_xy_yaw(self):
        """从雷达获取原始坐标和偏航角"""
        x_cm, y_cm, yaw = self.radar.rt_pose
        return -y_cm / 100, x_cm / 100, np.deg2rad(-yaw + 90)

    def get_xyyaw_relative(self):
        """获取相对于校准位置的坐标"""
        x, y, yaw = self.get_xy_yaw()
        return x - self.base_x, y - self.base_y, yaw

    def update_state(self, controller):
        """更新控制器状态"""
        x, y, yaw = self.get_xyyaw_relative()
        controller.update_state(x, y, yaw, self.speed)
        logger.debug(
            f"状态更新 - X: {x:.2f}m, Y: {y:.2f}m, Yaw: {np.rad2deg(yaw):.1f}°, Speed: {self.speed:.2f}m/s")

    def update_steer_and_speed(self, steer_rad: float, speed_mps: float):
        """更新舵机转向和电机速度"""
        # 计算轮速RPM
        def rpm(speed): return speed / self.WHEEL_PERIMETER * 60

        # 设置飞控参数
        self.fc.set_steer_and_speed(speed_mps, steer_rad)

        # 更新当前速度估计
        state = self.fc.get_state()
        v = state.motor_x_speed.value + 0.7
        self.speed += (v - self.speed) * 0.1

    def motor_lr(self, vx: float, vz: float):
        """
        阿克曼转向运动学计算
        :return: (左轮速度, 右轮速度) 单位 m/s
        """
        # 转向角度限幅
        vz = np.clip(vz, self.max_angle_l_rad, self.max_angle_r_rad)

        # 计算转弯半径
        if abs(vz) < 1e-3:  # 直行
            return vx, vx

        R = self.Axle_spacing / np.tan(vz) - 0.5 * self.Wheel_spacing

        # 计算轮速
        vl = vx * (R - 0.5 * self.Wheel_spacing) / R
        vr = vx * (R + 0.5 * self.Wheel_spacing) / R
        return vl, vr

    def get_pulse_from_deg(self, deg: float) -> int:
        """将角度转换为舵机脉冲宽度"""
        if deg >= self.deg_list[0]:
            return self.pulse_list[0]
        if deg <= self.deg_list[-1]:
            return self.pulse_list[-1]

        for i in range(len(self.deg_list) - 1):
            if self.deg_list[i + 1] <= deg <= self.deg_list[i]:
                return int(np.interp(deg,
                                     [self.deg_list[i + 1], self.deg_list[i]],
                                     [self.pulse_list[i + 1], self.pulse_list[i]]))

    def navigation_loop(self, target_x: float, target_y: float, is_return: bool = False):
        """
        执行单次导航循环
        :param target_x: 目标X坐标(m)
        :param target_y: 目标Y坐标(m)
        :param is_return: 是否为返程路径
        """
        # 获取当前位置
        x, y, yaw = self.get_xyyaw_relative()

        # 获取路径点
        enter_p, leave_p = self.rp.get_route(
            x, y, target_x, 0, target_y, 0, dl=self.dl)
        cx, cy, cyaw, ck = leave_p if is_return else enter_p

        # 初始化控制器
        controller = Controller(cx, cy, cyaw, ck,
                                self.target_speed, self.dl,
                                State(x, y, yaw, self.target_speed))
        self.update_state(controller)

        # 开始导航
        last_update = time.perf_counter()
        for steer, acc in controller.iter_output():
            # 计算控制量
            vel = controller.get_speed() * 1.2
            self.update_steer_and_speed(steer, vel)

            # 控制频率限制
            while time.perf_counter() - last_update < self.DT:
                time.sleep(0.02)

            last_update = time.perf_counter()
            self.update_state(controller)

        # 导航结束
        self.update_steer_and_speed(0, 0)
        time.sleep(1)
        logger.success(
            f"到达{'返程' if is_return else '去程'}目标点 ({target_x:.2f}, {target_y:.2f})")

    def run_mission(self, target_x: float, target_y: float):
        """执行完整往返任务"""
        try:
            # 去程导航
            self.navigation_loop(target_x, target_y, is_return=False)
            # 此处可添加任务逻辑

            # 返程导航
            self.navigation_loop(target_x, target_y, is_return=True)

        finally:
            self.update_steer_and_speed(0, 0)
            logger.info("任务完成")


if __name__ == "__main__":
    fc = FC_Client()
    fc.connect()
    fc.wait_for_connection()
    radar = LD_Radar()
    radar.start("/dev/ttyUSB0", "LD06")
    radar.start(subtask_skip=5)
    radar.start_resolve_pose(800, 0.7, 0.3, rotation_adapt=True)
    nav = DroneNavigationController()

    # 执行到目标点的往返任务
    target_x = 2.9232
    target_y = 1.8484
    nav.run_mission(target_x, target_y)
