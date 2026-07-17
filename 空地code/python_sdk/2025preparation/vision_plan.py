from ..vision.Vision_plus import debug_imshow, HighPrecisionFPS, get_ROI
from loguru import logger
import time
import cv2
from FlightController.Solutions.Navigation import Navigation
import numpy as np
import threading
timer = HighPrecisionFPS()


def id_function0(img):
    f, posx, posy, data = 0, 0, 0, 0
    return f, posx, posy, data


class id_mission (object):  # 定点识别类
    def __init__(self, *args, **kwargs) -> None:
        self.count = 0
        self.timeout = 0
        self.identify_status = False
        self.last_res = -1
        self.started = False
        self.res = -1
        self.cam: cv2.VideoCapture = kwargs["cam"]

    def id_function(self, img):
        debug_imshow(img, "Origin")
        logger.debug("正在识别")
        f, _, _, data = id_function0(img)
        if f:
            logger.info(f"data:{data}")
            return True, data
        return False, 0

    def camera_task(self):  # 到点后开识别
        self.count = 0
        self.timeout = 0
        while True:
            ret, img = self.cam.read()
            if not ret:
                logger.warning("没有图像")
                continue
            if self.identify_status:
                f, res = self.id_function(img)
                if f:
                    self.timeout = 0
                    if res == self.last_res:
                        self.count += 1
                    else:
                        self.count = 1
                    self.last_res = res
                else:
                    self.timeout += 1
                if self.count > 3:  # 此处改识别次数
                    self.count = 0
                    self.timeout = 0
                    self.identify_status = False
                    if self.started:
                        self.res = int(res)
                        logger.info("做任务了")
                        self.do_task()
                        self.next_point_event.set()
                elif self.timeout > 15:  # 此处为超时次数
                    self.timeout = 0
                    self.count = 0
                    self.identify_status = False
                    if self.started:
                        self.res = -1
                        logger.info("识别超时")
                        self.next_point_event.set()

            time.sleep(0.06)
            print(f"瞬时FPS: {timer.fps():.1f}")
            timer.start()

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def do_task(self):
        # 写识别到某结果后的任务
        if self.res == 1:
            logger.info("这是")


class target_mission(object):  # 寻找目标+识别类
    def __init__(self, *args, **kwargs) -> None:
        self.ob_count = 0  # 确认有
        self.res_count = 0  # 确认对
        self.timeout = 0  # 超时
        self.identify_status = False  # 识别状态
        self.last_res = -1
        self.started = False
        self.res = -1
        self.already = np.array()  # 记录已经查询过的点
        self.scope = 10  # 在多少范围内认为是同一个点
        self.allow_repeat = False  # 是否不需要查重
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.fc: FC_Like = kwargs["fc"]
        self.navi: Navigation = kwargs["navi"]
        self.use_pid = True  # 是否pid到位置#暂时都会用pid

    def id_function(self, img):
        debug_imshow(img, "Origin")
        logger.debug("正在识别")
        f, dx, dy, data = id_function0(img)
        if f:
            logger.info(f"data:{data}")
            return True, dx, dy, data

    def start_camera_task(self):
        threading.Thread(target=self.camera_task, daemon=True).start()

    def target(self):
        logger.info("找到东西，开始瞄准")
        K = 2  # 需要实际测量 K=此高度下某像素长度/对应的实际长度单位cm
        state = 0
        navi = self.navi
        fc = self.fc
        fc.set_rgb_led(255, 255, 0)
        while True:
            ret, img = self.cam.read()
            if not ret:
                logger.warning("没有图像")
                continue
            img = get_ROI(img, (0.27, 0.27, 0.46, 0.46))
            point = navi.current_point
            f, dx, dy, data = self.id_function(img)
            if f:
                if state == 0 and abs(dx) < 15 and abs(dy) < 15:  # 到位
                    self.res = data
                    logger.info(f"[MISSION] Reached point at {point}")
                    navi.set_height(100)
                    state = 1
                if state == 1 and abs(navi.current_height - navi.height_pid.setpoint) < 8:  # 到高
                    logger.info(f"[MISSION] Reached height at {point}")
                    fc.set_rgb_led(255, 0, 0)
                    state = 2
                if state == 2:  # 识别对了
                    if data == self.res:
                        state = 3
                    else:
                        self.res = data
                if state == 3:  # 做任务
                    logger.info("做任务了")
                    self.do_task()
                    if not self.allow_repeat:
                        self.already = np.append(
                            self.already, navi.current_point)
                    return
                to_point = (point[0] - dy / K, point[1] - dx / K)
                navi.direct_set_waypoint(to_point)
            time.sleep(0.08)

    def do_task(self):
        # 写识别到某结果后的任务
        if self.res == 1:
            logger.info("这是")

    def camera_task(self):
        self.ob_count = 0
        self.timeout = 0
        navi = self.navi
        while True:
            ret, img = self.cam.read()
            if not ret:
                logger.warning("没有图像")
                continue
            if self.identify_status:
                f, _, _, res = self.id_function(img)
                if f:
                    self.ob_count += 1
                else:
                    self.ob_count = 0
                if not self.allow_repeat:
                    curpoint = navi.current_point
                    if len(self.already) > 0:
                        # 计算所有点与新点的欧氏距离
                        distances = np.linalg.norm(
                            self.already - curpoint, axis=1)
                        if np.any(distances < self.scope):
                            self.ob_count = 0
                if self.ob_count > 1:  # 此处改确认存在次数
                    self.ob_count = 0
                    self.timeout = 0
                    self.identify_status = False
                    if self.started:
                        logger.info("停下导航")
                        self.navi.navigation_stop_here()
                        time.sleep(0.1)
                        self.res = int(res)
                        self.target()
                        self.navi.navigation_follow_trajectory(
                            self.navi.traj_list_before_stop, wait=False)
            time.sleep(0.06)
            print(f"瞬时FPS: {timer.fps():.1f}")
            timer.start()


class along_line_mission(object):
    def __init__(self, *args, **kwargs) -> None:
        self.ob_count = 0  # 确认有
        self.res_count = 0  # 确认对
        self.timeout = 0  # 超时
        self.identify_status = False  # 识别状态
        self.last_res = -1
        self.started = False
        self.res = -1
        self.already = np.array()  # 记录已经查询过的点
        self.scope = 10  # 在多少范围内认为是同一个点
        self.allow_repeat = False  # 是否不需要查重
        self.cam: cv2.VideoCapture = kwargs["cam"]
        self.fc: FC_Like = kwargs["fc"]
        self.navi: Navigation = kwargs["navi"]
        self.use_pid = True  # 是否pid到位置#暂时都会用pid
