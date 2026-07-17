import numpy as np
from configManager import ConfigManager
from FlightController import FC_Client, FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Solutions.Navigation import Navigation

cfg = ConfigManager(section="mission")
fc = FC_Controller()
fc.start_listen_serial()
fc.wait_for_connection()
t265 = T265()
t265.start()
t265.hardware_reset()
radar = LD_Radar()
radar.start("/dev/ttyUSB0", "LD06")
navi = Navigation(fc, radar, t265)
navi.start()
navi.switch_navigation_mode("fusion")

while True:
    act = input("Enter action: ")
    if act == "?":
        print(f"Current point: {navi.current_point}")
        print(f"Radar pose: {radar.rt_pose}")
    elif act == "b":
        point = navi.calibrate_basepoint()
        print(f"Basepoint: {point}")
        cfg.set("point-base", point)
    elif act.isdigit():
        idx = int(act)
        point = navi.current_point
        print(f"Point-{idx}: {point}")
        cfg.set(f"point-{idx}", point)
    elif act == "t":
        idx = int(act)
        get = input(f"Enter x,y for target-{idx}: ")
        x, y = get.split(",")
        cfg.set(f"target-{idx}", np.array([int(x), int(y)]))
