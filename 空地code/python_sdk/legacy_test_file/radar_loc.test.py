import time

import cv2
import numpy as np
from FlightController import FC_Client, FC_Controller
from FlightController.Components import LD_Radar, Point_2D
from FlightController.Solutions.Radar_SLAM import *
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *

radar = LD_Radar()
# fc = FC_Controller()
# fc.start_listen_serial(print_state=True)
# fc = FC_Client()
# fc.connect("fc-ubuntu", print_state=False)
# fc.wait_for_connection()
radar.start()
# radar.start(fc)

fps = fps_counter()
time.sleep(2)
radar.subtask_event.wait(1)
pts = radar.map.output_points(0.1, False)
icpm = ICPM(pts)
rot = 0.0
while True:
    radar.subtask_event.wait(1)
    fps.update()
    t0 = time.perf_counter()
    img = radar.map.output_cloud(0.1, 800, rot_angle=rot)
    x, y, yaw = radar_resolve_rt_pose(img, 1 ,1)
    # img = radar.map.output_polyline_cloud(0.1, 800, thickness=1, draw_outside=False)
    # x, y, yaw = radar_resolve_rt_pose(img, 1, skip_er=True, skip_di=True)
    t1 = time.perf_counter()
    print(f"fps: {fps.fps:.2f}", f"cost: {t1-t0:.6f}", f"pose: {x}, {y}, {yaw}, {rot}")
    # pts = radar.map.output_points(0.1, False, rot_angle=rot)
    # ret = icpm.match(pts, True, debug_size=1000)
    # t1 = time.perf_counter()
    # print(
    #     f"fps: {fps.fps:.2f}",
    #     f"cost: {t1-t0:.6f}",
    #     f"ret: {ret}",
    #     f"pose: {icpm.translation}, {icpm.rotation_as_euler}, {rot}",
    # )
    # yaw = icpm.rotation_as_euler
    if (key := cv2.waitKey(1) & 0xFF) == ord("q"):
        break
    elif key == ord("s"):
        icpm.update_template(pts)
    rot += (yaw)/2 if yaw is not None else 0
