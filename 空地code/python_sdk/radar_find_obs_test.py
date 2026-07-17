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
cv2.namedWindow("Radar", cv2.WINDOW_NORMAL)
while True:

    img = radar.map.output_cloud(0.1, 2000)
    debug_imshow(img, "Radar")
    point = radar_resolve_obs(img, True, True)
    for i in range(len(point)):
        x = int(point[i][0])
        y = int(point[i][1])
    # point = np.array(point)
        logger.debug(f"loc: {x}, {y}")
    if cv2.waitKey(1) & 0xFF == ord('q'):
            break
