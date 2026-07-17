import math
import os
import threading as t
import time
from typing import Optional

import cv2
import numpy as np
from djitellopy import Tello
from simple_pid import PID

# ip = "192.168.10.1"
# ip = "192.168.3.211"
ip = "192.168.6.211"
stop_event = t.Event()
key_queue: list[int] = []
drone = Tello(host=ip)

lmouse_down = False
lmouse_down_pos = (0, 0)
rmouse_down = False
rmouse_down_pos = (0, 0)
double_down = False
double_down_pos = (0, 0)
mouse_pos = (0, 0)
mouse_pos = (0, 0)
takeoff = False
v_list = [0, 0, 0, 0]
snr: int = 0
last_snr_query_time: float = 0
engage_trigger = False
engage_time = 0
mission_pad_detection = False
lockon = False
lockon_kcf: Optional["KCFTracker"] = None
lockon_init_pos = ((0, 0), (0, 0))
lockon_pid_yaw = PID(0.3, 0.0, 0, setpoint=0, output_limits=(-100, 100))
lockon_pid_height = PID(0.3, 0.0, 0, setpoint=0, output_limits=(-100, 100))


class KCFTracker:
    padding = 2  # padding：扩展跟踪框的大小，以适应目标的运动
    sigma = 0.2  # sigma：高斯核的标准差，用于计算核相关矩阵
    interp_factor = 0.075  # interp_factor：插值因子，用于更新核相关矩阵
    output_sigma_factor = 0.1  # output_sigma_factor：输出标准差的缩放因子，用于计算核相关矩阵
    cell_size = 4  # cell_size：每个单元格的大小，用于计算HOG特征
    update_interval = 10  # update_interval：更新跟踪器的间隔帧数

    def __init__(self, image, p1, p2):
        _roi = (p1[0], p1[1], p2[0] - p1[0], p2[1] - p1[1])
        self._tracker = cv2.TrackerKCF_create()  # type: ignore
        self._tracker.init(image, _roi)
        self._last_tracker = None  # last_tracker：上一次的跟踪器
        self._update_counter = 0  # update_counter：更新跟踪器的计数器
        self._init_roi_size = _roi[2] * _roi[3]  # init_roi_size：初始跟踪框的大小

    def update(self, image) -> Optional[tuple[tuple[int, int], tuple[int, int]]]:
        self._update_counter += 1
        ok, roi = self._tracker.update(image)
        if ok:
            p1 = (int(roi[0]), int(roi[1]))
            p2 = (int(roi[0] + roi[2]), int(roi[1] + roi[3]))
            if self._update_counter >= self.update_interval and 0.9 < roi[2] * roi[3] / self._init_roi_size < 1.1:
                self._update_counter = 0
                self._last_tracker = self._tracker
                self._tracker = cv2.TrackerKCF_create()  # type: ignore
                self._tracker.init(image, roi)
            return p1, p2
        else:
            if self._update_counter < self.update_interval / 2 and self._last_tracker is not None:
                # 如果跟踪失败，则尝试fallback到上一次的跟踪器
                ok, roi = self._last_tracker.update(image)
                if ok:
                    p1 = (int(roi[0]), int(roi[1]))
                    p2 = (int(roi[0] + roi[2]), int(roi[1] + roi[3]))
                    self._tracker = self._last_tracker
                    self._last_tracker = None
                    return p1, p2
            return None


def on_mouse(event, x, y, flags, param):
    global lmouse_down, lmouse_down_pos, rmouse_down, rmouse_down_pos, double_down, double_down_pos
    global mouse_pos, takeoff, v_list, engage_trigger, engage_time, lockon, lockon_kcf, lockon_init_pos
    mouse_pos = (x, y)
    if event == cv2.EVENT_LBUTTONDOWN:
        lmouse_down = True
        lmouse_down_pos = (x, y)
    elif event == cv2.EVENT_LBUTTONUP:
        lmouse_down = False
    elif event == cv2.EVENT_RBUTTONDOWN:
        rmouse_down = True
        rmouse_down_pos = (x, y)
    elif event == cv2.EVENT_RBUTTONUP:
        rmouse_down = False
    elif event == cv2.EVENT_MBUTTONDOWN:
        if not takeoff:
            key_queue.append(ord("t"))
            takeoff = True
        else:
            key_queue.append(ord("l"))
            takeoff = False
    elif event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0 and not (takeoff or engage_trigger):
            engage_trigger = True
            engage_time = time.time()
            v_list = [100, -100, -100, -100]
            print("engage trigger on")
            takeoff = True
        elif flags < 0:
            key_queue.append(8)
    if lmouse_down and rmouse_down:
        if not double_down:
            if not lockon:
                double_down = True
                double_down_pos = (x, y)
            else:
                lockon = False
                lmouse_down = False
                rmouse_down = False
                double_down_pos = (-1, -1)
                print("lockon off")
                lockon_kcf = None
                v_list[2] = 0
                v_list[3] = 0
    else:
        if double_down:
            double_down = False
            lmouse_down = False
            rmouse_down = False
            if double_down_pos != (-1, -1) and double_down_pos != mouse_pos:
                lockon = True
                print(f"lockon: {double_down_pos} -> {mouse_pos}")
                lockon_init_pos = (double_down_pos, mouse_pos)


def draw_lockon(img: np.ndarray, p1: tuple[int, int], p2: tuple[int, int]) -> None:
    L_C = (255, 0, 0)
    cv2.rectangle(img, p1, p2, L_C, 2)


def update_lockon(img: np.ndarray) -> None:
    global lockon_kcf
    L_C = (255, 0, 0)
    width, height = img.shape[1], img.shape[0]
    info = lambda str, color: cv2.putText(
        img,
        str,
        (width // 2 - cv2.getTextSize(str, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0][0] // 2, height // 4),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        color,
        2,
    )
    if lockon:
        if lockon_kcf is None:
            try:
                lockon_kcf = KCFTracker(img, lockon_init_pos[0], lockon_init_pos[1])
                info("LOCKON INIT", (0, 255, 255))
                lockon_pid_yaw.reset()
                lockon_pid_height.reset()
            except:
                info("LOCKON FAIL", (0, 0, 255))
                lockon_kcf = None
        else:
            try:
                p = lockon_kcf.update(img)
            except:
                info("LOCKON FAIL", (0, 0, 255))
                p = None
            if p is None:
                info("LOCKON LOST", (0, 0, 255))
                v_list[2] = 0
                v_list[3] = 0
                lockon_pid_yaw.reset()
                lockon_pid_height.reset()
            else:
                info("LOCKON", (0, 255, 0))
                center_pos = ((p[0][0] + p[1][0]) // 2, (p[0][1] + p[1][1]) // 2)
                cv2.rectangle(img, p[0], p[1], L_C, 2)
                cv2.line(img, (center_pos[0], center_pos[1]), (width // 2, height // 2), L_C, 2)
                lockon_pid_yaw.setpoint = width // 2
                lockon_pid_height.setpoint = height // 2
                yout = lockon_pid_yaw(center_pos[0])
                hout = lockon_pid_height(center_pos[1])
                if yout is not None and hout is not None:
                    v_list[2] = round(-yout)
                    v_list[3] = round(hout)


def action_task():
    global takeoff, snr, last_snr_query_time, v_list, engage_trigger, engage_time, mission_pad_detection
    while not stop_event.is_set():
        try:
            # print(f"v_list: {v_list}")
            if engage_trigger and time.time() - engage_time > 0.5:
                v_list = [0, 0, 0, 0]
                engage_trigger = False
                print("engage trigger off")
            drone.send_rc_control(*v_list)
            if len(key_queue) > 0:
                key = key_queue.pop(0)
                print(f"key: {key}")
                if key == ord("t"):
                    drone.takeoff()
                    takeoff = drone.is_flying
                elif key == ord("l"):
                    drone.land()
                    takeoff = drone.is_flying
                elif key == ord("o"):
                    drone.turn_motor_on()
                elif key == ord("p"):
                    drone.turn_motor_off()
                elif key == ord("w"):
                    drone.flip_forward()
                elif key == ord("s"):
                    drone.flip_back()
                elif key == ord("a"):
                    drone.flip_left()
                elif key == ord("d"):
                    drone.flip_right()
                elif key == ord("m"):
                    mission_pad_detection = not mission_pad_detection
                    if mission_pad_detection:
                        drone.enable_mission_pads()
                        time.sleep(0.5)
                        drone.set_mission_pad_detection_direction(0)
                    else:
                        drone.disable_mission_pads()
                elif key == 8:  # backspace
                    drone.emergency()
                    takeoff = False
            if time.time() - last_snr_query_time > 10:
                snr = int(drone.query_wifi_signal_noise_ratio())
                last_snr_query_time = time.time()
            time.sleep(0.05)
        except Exception as e:
            print(e)


def draw_joystick(
    img: np.ndarray, center_pos: tuple[int, int], pos: tuple[int, int], color: tuple[int, int, int] = (255, 255, 255)
) -> tuple[int, int]:
    R = 100
    pos = (
        min(max(pos[0], center_pos[0] - R), center_pos[0] + R),
        min(max(pos[1], center_pos[1] - R), center_pos[1] + R),
    )
    cv2.rectangle(img, (center_pos[0] - R, center_pos[1] - R), (center_pos[0] + R, center_pos[1] + R), color, 2)
    dark_color = tuple([int(c * 0.5) for c in color])
    cv2.line(img, center_pos, pos, color, 2)
    cv2.circle(img, center_pos, 5, dark_color, -1)
    cv2.circle(img, pos, 5, color, -1)
    x_diff = round((pos[0] - center_pos[0]) / R * 100)
    y_diff = round((pos[1] - center_pos[1]) / R * 100)
    return (x_diff, y_diff)


class fps_counter:
    def __init__(self, max_sample=60) -> None:
        self.t = time.perf_counter()
        self.max_sample = max_sample
        self.t_list: list[float] = []

    def update(self) -> None:
        now = time.perf_counter()
        self.t_list.append(now - self.t)
        self.t = now
        if len(self.t_list) > self.max_sample:
            self.t_list.pop(0)

    @property
    def fps(self) -> float:
        length = len(self.t_list)
        sum_t = sum(self.t_list)
        if length == 0:
            return 0.0
        else:
            return length / sum_t


blk1: Optional[np.ndarray] = None
blk2: Optional[np.ndarray] = None
fpsc = fps_counter()
acc_z_calc = 10.0
LPR = 0.2  # Low Pass Ratio
pitch, roll, yaw = 0.0, 0.0, 0.0
vel_x, vel_y, vel_z = 0.0, 0.0, 0.0
accel_x, accel_y, accel_z = 0.0, 0.0, 0.0
baro, tof, hei = 0.0, 0.0, 0.0


def draw_gauge(img: np.ndarray) -> np.ndarray:
    global blk1, blk2, acc_z_calc
    global pitch, roll, yaw
    global vel_x, vel_y, vel_z
    global accel_x, accel_y, accel_z
    global baro, tof, hei

    def continue_add(old, new):
        if abs(old - new) > 300:
            if old > new:
                value = old * (1 - LPR) + (new + 360) * LPR
            else:
                value = old * (1 - LPR) + (new - 360) * LPR
            if value > 180:
                value -= 360
            elif value < -180:
                value += 360
            return value
        return old * (1 - LPR) + new * LPR

    pitch, roll, yaw = (
        pitch * (1 - LPR) + drone.get_pitch() * LPR,
        continue_add(roll, drone.get_roll()),
        continue_add(yaw, drone.get_yaw()),
    )
    vel_x, vel_y, vel_z = (
        vel_x * (1 - LPR) + drone.get_speed_x() * LPR,
        vel_y * (1 - LPR) + drone.get_speed_y() * LPR,
        vel_z * (1 - LPR) + drone.get_speed_z() * LPR,
    )
    accel_x, accel_y, accel_z = (
        accel_x * (1 - LPR) + drone.get_acceleration_x() / 100 * LPR,
        accel_y * (1 - LPR) + drone.get_acceleration_y() / 100 * LPR,
        accel_z * (1 - LPR) + (drone.get_acceleration_z() / 100 - acc_z_calc) * LPR,
    )
    baro, tof, hei = (
        baro * (1 - LPR) + drone.get_barometer() * LPR,
        tof * (1 - LPR) + drone.get_distance_tof() * LPR,
        hei * (1 - LPR) + drone.get_height() * LPR,
    )

    acc_z_calc += accel_z * 0.1
    width, height = img.shape[1], img.shape[0]
    ########## pitch ###########
    P_LW = 40  # line width
    P_H = 400  # height
    P_S = 8  # space
    P_I = 5  # interval
    P_C = (120, 255, 0)  # color
    P_XO = -300  # x offset
    ########## height ###########
    H_LW = 40  # line width
    H_H = 400  # height
    H_S = 4  # space
    H_I = 10  # interval
    H_C = (120, 255, 0)  # color
    H_XO = 300
    H_VR = 10
    H_AR = 30
    ########## yaw ###########
    Y_LW = 20  # line width
    Y_W = 400  # width
    Y_S = 5  # space
    Y_I = 10  # interval
    Y_C = (0, 220, 255)  # color
    Y_YO = -255  # y offset
    ########## roll ###########
    R_LW = 20  # line width
    R_W = 400  # width
    R_S = 5  # space
    R_I = 10  # interval
    R_C = (0, 220, 255)  # color
    R_YO = 255  # y offset
    ########## acc ###########
    A_R = 10
    A_LW = 10  # line width
    A_C = (0, 255, 255)  # color
    ########## vel ###########
    V_R = 10
    V_LW = 20  # line width
    V_C = (0, 255, 0)  # color
    if blk1 is None or blk1.shape[0] != height or blk1.shape[1] != width:
        blk1 = np.zeros_like(img)
        length = min(height, width) // 3
        cv2.line(
            blk1,
            ((width - length) // 2, height // 2),
            ((width + length) // 2, height // 2),
            (255, 255, 255),
            1,
        )
        cv2.line(
            blk1,
            (width // 2, (height - length) // 2),
            (width // 2, (height + length) // 2),
            (255, 255, 255),
            1,
        )
    img = cv2.addWeighted(img, 1.0, blk1, 0.5, 1)
    if blk2 is None or blk2.shape[0] != height or blk2.shape[1] != width:
        blk2 = np.zeros_like(img)
        cv2.rectangle(
            blk2,
            (width // 2 + P_XO - P_LW // 2 - 40, height // 2 - P_H // 2),
            (width // 2 + P_XO + P_LW // 2, height // 2 + P_H // 2),
            (255, 255, 255),
            -1,
        )
        cv2.rectangle(
            blk2,
            (width // 2 + H_XO - H_LW // 2, height // 2 - H_H // 2),
            (width // 2 + H_XO + H_LW // 2 + 40, height // 2 + H_H // 2),
            (255, 255, 255),
            -1,
        )
        cv2.rectangle(
            blk2,
            (width // 2 + Y_W // 2 + 10, height // 2 + Y_YO - Y_LW // 2 - 30),
            (width // 2 - Y_W // 2 - 10, height // 2 + Y_YO + Y_LW // 2),
            (255, 255, 255),
            -1,
        )
        cv2.rectangle(
            blk2,
            (width // 2 + R_W // 2 + 10, height // 2 + R_YO - R_LW // 2),
            (width // 2 - R_W // 2 - 10, height // 2 + R_YO + R_LW // 2 + 30),
            (255, 255, 255),
            -1,
        )
    img = cv2.addWeighted(img, 1.0, blk2, -0.2, 1)
    ########## pitch ###########
    for p in range(-90, 90 + P_I, P_I):
        if -P_H / 2 < (p - pitch) * P_S < P_H / 2:
            scale = 2 if p == 0 else 1
            cv2.line(
                img,
                (width // 2 - P_LW // 2 + P_XO, round(height // 2 - (p - pitch) * P_S)),
                (width // 2 + P_LW // 2 + P_XO, round(height // 2 - (p - pitch) * P_S)),
                P_C,
                1,
            )
            cv2.line(
                img,
                (width // 2 - P_LW // 2 + P_XO, height // 2),
                (width // 2 + P_LW // 2 + P_XO, height // 2),
                P_C,
                2,
            )
            cv2.putText(
                img,
                f"{p:3d}",
                (width // 2 - P_LW // 2 - 35 + P_XO, round(height // 2 - (p - pitch) * P_S) + 5),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                P_C,
                scale,
            )
    cv2.putText(
        img,
        f"PITCH",
        (width // 2 - P_LW // 2 + 45 + P_XO, height // 2 + 5),
        cv2.FONT_HERSHEY_PLAIN,
        1,
        P_C,
        1,
    )
    ########## height ###########
    for h in range(
        round(hei) // H_I * H_I - round(H_H / H_S * H_I), round(hei) // H_I * H_I + round(H_H / H_S * H_I), H_I
    ):
        if -H_H / 2 < (h - hei) * H_S < H_H / 2 - 36:
            scale = 2 if h == 0 else 1
            cv2.line(
                img,
                (width // 2 - H_LW // 2 + H_XO, round(height // 2 - (h - hei) * H_S)),
                (width // 2 + H_LW // 2 + H_XO, round(height // 2 - (h - hei) * H_S)),
                H_C,
                1,
            )
            cv2.line(
                img,
                (width // 2 - H_LW // 2 + H_XO, height // 2),
                (width // 2 + H_LW // 2 + H_XO, height // 2),
                H_C,
                2,
            )
            cv2.putText(
                img,
                f"{h:<6d}",
                (width // 2 + H_LW // 2 + 5 + H_XO, round(height // 2 - (h - hei) * H_S) + 5),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                H_C,
                scale,
            )
    cv2.line(
        img,
        (width // 2 - H_LW // 2 + H_XO, height // 2),
        (width // 2 - H_LW // 2 + H_XO, height // 2 - round(H_VR * vel_z)),
        V_C,
        1,
    )
    cv2.line(
        img,
        (width // 2 + H_LW // 2 + H_XO + 1, height // 2),
        (width // 2 + H_LW // 2 + H_XO + 1, height // 2 - round(H_AR * accel_z)),
        A_C,
        1,
    )
    cv2.putText(
        img,
        f"ATI",
        (width // 2 + H_LW // 2 - 70 + H_XO, height // 2 + 5),
        cv2.FONT_HERSHEY_PLAIN,
        1,
        H_C,
        1,
    )
    cv2.putText(
        img,
        f"R:{tof/100:<7.2f}",
        (width // 2 + H_XO - 15, height // 2 - H_H // 2 + 15),
        cv2.FONT_HERSHEY_PLAIN,
        1,
        H_C,
        1,
    )
    cv2.putText(
        img,
        f"B:{baro/100:<7.2f}",
        (width // 2 + H_XO - 15, height // 2 - H_H // 2 + 30),
        cv2.FONT_HERSHEY_PLAIN,
        1,
        H_C,
        1,
    )

    ########## yaw ###########
    def map_180(yaw):
        if yaw < -180:
            yaw += 360
        elif yaw > 180:
            yaw -= 360
        return yaw

    for y in range(180, -180, -Y_I):
        if -Y_W / 2 < map_180(y - yaw) * Y_S < Y_W / 2:
            scale = 2 if y % (Y_I * 3) == 0 else 1
            cv2.line(
                img,
                (width // 2 + round(map_180(y - yaw) * Y_S), height // 2 - Y_LW // 2 + Y_YO),
                (width // 2 + round(map_180(y - yaw) * Y_S), height // 2 + Y_LW // 2 + Y_YO),
                Y_C,
                1,
            )
            cv2.line(
                img,
                (width // 2, height // 2 - Y_LW // 2 + Y_YO),
                (width // 2, height // 2 + Y_LW // 2 + Y_YO),
                Y_C,
                2,
            )
            if y == 0:
                x_o = 29
            elif y >= 100:
                x_o = 22
            else:
                x_o = 26
            cv2.putText(
                img,
                f"{y:4d}",
                (width // 2 + round(map_180(y - yaw) * Y_S) - x_o, height // 2 - Y_LW // 2 - 10 + Y_YO),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                Y_C,
                scale,
            )
    cv2.putText(
        img,
        f"YAW",
        (width // 2 - 15, height // 2 - Y_LW // 2 + 40 + Y_YO),
        cv2.FONT_HERSHEY_PLAIN,
        1,
        Y_C,
        1,
    )
    ########## roll ###########
    for r in range(180, -180, -R_I):
        if -R_W / 2 < map_180(r - roll) * R_S < R_W / 2:
            scale = 2 if r == 0 else 1
            cv2.line(
                img,
                (width // 2 + round(map_180(r - roll) * R_S), height // 2 - R_LW // 2 + R_YO),
                (width // 2 + round(map_180(r - roll) * R_S), height // 2 + R_LW // 2 + R_YO),
                R_C,
                1,
            )
            cv2.line(
                img,
                (width // 2, height // 2 - R_LW // 2 + R_YO),
                (width // 2, height // 2 + R_LW // 2 + R_YO),
                R_C,
                2,
            )
            if r == 0:
                x_o = 29
            elif y >= 100:
                x_o = 22
            else:
                x_o = 26
            cv2.putText(
                img,
                f"{r:4d}",
                (width // 2 + round(map_180(r - roll) * R_S) - x_o, height // 2 - R_LW // 2 + 40 + R_YO),
                cv2.FONT_HERSHEY_PLAIN,
                1,
                R_C,
                scale,
            )
    cv2.putText(
        img,
        "ROLL",
        (width // 2 - 18, height // 2 - R_LW // 2 - 10 + R_YO),
        cv2.FONT_HERSHEY_PLAIN,
        1,
        R_C,
        1,
    )
    ########## vel ###########
    v_pos = (width // 2 + round(vel_x * V_R), height // 2 + round(vel_y * V_R))
    cv2.line(img, (v_pos[0] - V_LW, v_pos[1]), (v_pos[0] + V_LW, v_pos[1]), V_C, 1)
    cv2.line(img, (v_pos[0], v_pos[1] - V_LW), (v_pos[0], v_pos[1] + V_LW), V_C, 1)
    ########## acc ###########
    a_pos = (width // 2 - round(accel_y * A_R), height // 2 + round(accel_x * A_R))
    cv2.line(img, (a_pos[0] - A_LW, a_pos[1]), (a_pos[0] + A_LW, a_pos[1]), A_C, 1)
    cv2.line(img, (a_pos[0], a_pos[1] - A_LW), (a_pos[0], a_pos[1] + A_LW), A_C, 1)
    return img


def draw_side_info(img, x, y, str, warn=False, right=False):
    def get_str_width(str):
        return cv2.getTextSize(str, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]

    if right:
        width = img.shape[1]
        x = width - x - get_str_width(str)
    cv2.putText(
        img,
        str,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 255, 255) if not warn else (0, 255, 255),
        1,
    )


def draw_info(img: np.ndarray):
    img = draw_gauge(img)
    width, height = img.shape[1], img.shape[0]

    draw_side_info(img, 8, 22, f"RES {width}x{height}")
    draw_side_info(img, 8, 42, f"FPS {fpsc.fps:.1f}")
    draw_side_info(img, 7, 62, f"SNR {snr}", snr < 50)
    bat = drone.get_battery()
    draw_side_info(img, 9, 82, f"BAT {bat}%", bat < 20)
    tmp = drone.get_temperature()
    draw_side_info(img, 7, 102, f"TMP {tmp}C", tmp > 80)
    draw_side_info(img, 11, 122, f"FLT {drone.get_flight_time()}")
    draw_side_info(img, 11, 142, "ARMED" if takeoff else "DISARMED", takeoff)
    if mission_pad_detection:
        draw_side_info(img, 11, 162, "MISN-PAD:")
        id = drone.get_mission_pad_id()
        if id > 0:
            draw_side_info(img, 11, 182, f"ID= {drone.get_mission_pad_id()}")
            draw_side_info(img, 11, 202, f"X= {drone.get_mission_pad_distance_x()}")
            draw_side_info(img, 11, 222, f"Y= {drone.get_mission_pad_distance_y()}")
            draw_side_info(img, 11, 242, f"Z= {drone.get_mission_pad_distance_z()}")
        else:
            draw_side_info(img, 11, 182, "UNDETECTED")
    return img


def draw_vlist(img: np.ndarray):
    draw_side_info(img, 8, 22, "OUT", right=True)
    for i, v in enumerate(v_list):
        draw_side_info(img, 8, 42 + i * 20, f"{v:3d}", right=True)


drone.connect()
# drone.streamoff()
try:
    drone.set_video_resolution(drone.RESOLUTION_720P)
    drone.set_video_fps(drone.FPS_30)
    drone.set_video_bitrate(drone.BITRATE_4MBPS)
except:
    pass
try:
    frame_read = drone.get_frame_read()
except:
    drone.background_frame_read = None
    drone.streamon()
    frame_read = drone.get_frame_read()
thread = t.Thread(target=action_task, daemon=True)
thread.start()
img = np.zeros((720, 960, 3), np.uint8)
cv2.namedWindow("drone", cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
cv2.setMouseCallback("drone", on_mouse)  # type: ignore
drone.no_return = True
last_image_id = -1
while not stop_event.is_set():
    get_img = frame_read.frame
    if get_img is not None and id(get_img) != last_image_id:
        last_image_id = id(get_img)
        fpsc.update()
        img = cv2.cvtColor(get_img, cv2.COLOR_RGB2BGR)
        update_lockon(img)
    show_img = img.copy()
    show_img = draw_info(show_img)
    if double_down and not lockon:
        draw_lockon(show_img, double_down_pos, mouse_pos)
    if not engage_trigger:
        if lmouse_down and (not double_down):
            x, y = draw_joystick(show_img, lmouse_down_pos, mouse_pos, (0, 255, 0))
            v_list[0] = x
            v_list[1] = -y
        else:
            v_list[0] = 0
            v_list[1] = 0
        if rmouse_down and (not double_down):
            x, y = draw_joystick(show_img, rmouse_down_pos, mouse_pos, (0, 0, 255))
            v_list[2] = -y
            v_list[3] = x
        elif not lockon:
            v_list[2] = 0
            v_list[3] = 0
    draw_vlist(show_img)
    cv2.imshow("drone", show_img)
    key = cv2.waitKey(10) & 0xFF
    if key == 27:  # ESC
        stop_event.set()
        break
    elif key != 255:
        print(f"read key: {key}")
        key_queue.append(key)
drone.no_return = False
cv2.destroyAllWindows()
if drone.is_flying:
    drone.land()
thread.join()
