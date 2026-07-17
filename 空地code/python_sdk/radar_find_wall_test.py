from FlightController.Components.LDRadar_Driver import LD_Radar
from FlightController.Solutions.Radar_find_obs import radar_resolve_obs, radar_resolve_wall
from loguru import logger
import cv2
import numpy as np
from vision.Vision_plus import debug_imshow
KERNAL_DI = 9
KERNAL_ER = 5
HOUGH_THRESHOLD = 60
MIN_LINE_LENGTH = 60
MAX_LINE_GAP = 200
kernel_di = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE, (KERNAL_DI, KERNAL_DI))
kernel_er = cv2.getStructuringElement(
    cv2.MORPH_ELLIPSE, (KERNAL_ER, KERNAL_ER))
radar = LD_Radar()
radar.start()
logger.info("雷达已启动")
# radar.register_map_func(
#     radar.map.find_nearest_with_ext_point_opt, from_=45, to_=135, num=3)

while True:
    # 获取最新点云
    img = radar.map.output_cloud()
    debug_imshow(img, "Origin")
    detected_walls = radar_resolve_wall(
           img, 150, 1, 2, 60, 10, True)
    if len(detected_walls) == 0:
            print("未检测到任何墙壁。")
    elif len(detected_walls) == 1:
            print("检测到1条墙壁：")
            wall_angle, wall_distance, wall_segment = detected_walls[0]
            print(f"  - 距离: {wall_distance:.2f} 像素, 角度: {wall_angle:.2f} 度")
            print(
                f"  - 转换后线段端点: ({wall_segment[0]:.2f}, {wall_segment[1]:.2f}) -> ({wall_segment[2]:.2f}, {wall_segment[3]:.2f})")
    elif len(detected_walls) >= 2:
            print("检测到2条墙壁 (最近和次近)：")

            # 最近的墙壁
            wall1_angle, wall1_distance, wall1_segment = detected_walls[0]
            print(f"  - 第一条 (最近) 墙壁:")
            print(
                f"    - 距离: {wall1_distance:.2f} 像素, 角度: {wall1_angle:.2f} 度")
            print(
                f"    - 转换后线段端点: ({wall1_segment[0]:.2f}, {wall1_segment[1]:.2f}) -> ({wall1_segment[2]:.2f}, {wall1_segment[3]:.2f})")

            # 次近的墙壁
            wall2_angle, wall2_distance, wall2_segment = detected_walls[1]
            print(f"  - 第二条 (次近) 墙壁:")
            print(
                f"    - 距离: {wall2_distance:.2f} 像素, 角度: {wall2_angle:.2f} 度")
            print(
                f"    - 转换后线段端点: ({wall2_segment[0]:.2f}, {wall2_segment[1]:.2f}) -> ({wall2_segment[2]:.2f}, {wall2_segment[3]:.2f})")

    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
