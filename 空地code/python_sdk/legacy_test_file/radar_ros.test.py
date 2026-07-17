import time

import cv2
from FlightController import FC_Client, FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Components.RosNode import Laser2PointCloudNode
from FlightController.Solutions.Radar_SLAM import *
from loguru import logger

radar = LD_Radar()
# fc = FC_Controller()
# fc.start_listen_serial(print_state=True)
fc = FC_Client()
fc.connect()
fc.wait_for_connection()
radar.start(fc)
t265 = T265()
t265.start()
time.sleep(1)
node = Laser2PointCloudNode(radar, t265)
node.start()
# try:
#     radar.start("COM28", "LD06")
# except:
#     radar.start("/dev/ttyUSB0", "LD06")
# radar.register_map_func(radar.map.find_nearest_with_ext_point_opt, from_=45, to_=135, num=3)
# radar.show_radar_map()
# radar.show_radar_map(add_p_func=radar.map.find_two_point_with_given_distance, from_=0, to_=90, distance=100)
# radar.show_radar_map()
# fc.close(True)
while True:
    time.sleep(1)
