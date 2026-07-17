import os
import sys
from time import sleep, time
from typing import Any

from config_manager import ConfigManager
from FlightController import FC_Client, FC_Controller
from FlightController.Components import LD_Radar
from FlightController.Components.RealSense import T265
from FlightController.Components.RosManager import RosManager
from FlightController.Components.RosMapper import RosMapper
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.UartScreen import UARTScreen
from FlightController.Solutions.Navigation import Navigation
from FlightController.Solutions.Vision import open_camera
from loguru import logger


def self_reboot():
    logger.info("[MANAGER] Manager Restarting")
    try:
        from win32api import GetShortPathName as ShortName

        os.execl(ShortName(sys.executable), ShortName(sys.executable), *sys.argv)
    except:
        os.execl(sys.executable, sys.executable, *sys.argv)


fc = FC_Client()
try:
    fc.connect()
    assert fc.wait_for_connection(4)
except:
    fc.close()
    logger.warning("[MANAGER] Manager Connecting Failed, Switching to Local Mode")
    try:
        fc = FC_Controller()  # type: ignore
        fc.start_listen_serial(print_state=False)
        assert fc.wait_for_connection(4)
    except:
        logger.error("[MANAGER] Local Mode Failed, Restarting")
        sleep(1)
        self_reboot()

fc.set_action_log(False)
fc.event.key_short.clear()
fc.event.key_double.clear()
fc.event.key_long.clear()
fc.set_indicator_led(0, 0, 0)

try:
    radar = LD_Radar()
    radar.start("ros")
    # sleep(2)
    # # assert radar.connected
except Exception as e:
    logger.warning(f"[MANAGER] Radar Connecting Failed {e}")
    while True:
        fc.set_indicator_led(255, 0, 0)
        sleep(0.5)
        fc.set_indicator_led(0, 0, 0)
        sleep(0.5)
        if fc.event.key_short.is_set():
            fc.close()
            self_reboot()

try:
    rs = T265("ros")
    rs.hardware_reset()
    rs.start(print_update=False)
except:
    logger.warning("[MANAGER] RealSense(T265) Connecting Failed")
    while True:
        fc.set_indicator_led(0, 255, 255)
        sleep(0.5)
        fc.set_indicator_led(0, 0, 0)
        sleep(0.5)
        if fc.event.key_short.is_set():
            fc.close()
            self_reboot()

try:
    cam, id = open_camera()
    logger.info(f"[MANAGER] Camera {id} Opened")
except:
    logger.warning("[MANAGER] Camera Opening Failed")
    while True:
        fc.set_indicator_led(255, 255, 0)
        sleep(0.5)
        fc.set_indicator_led(0, 0, 0)
        sleep(0.5)
        if fc.event.key_short.is_set():
            fc.close()
            radar.stop()
            rs.stop()
            self_reboot()

screen = UARTScreen(fc)
mapper = RosMapper()
navi = Navigation(
    fc=fc,
    rs=rs,
    radar=radar,
    mapper=mapper,
)
RosNodeRunner().add_nodes().run()

############################## 参数 ##############################
cfg = ConfigManager()
############################## 初始化 ##############################
logger.info("[MANAGER] Self-Checking Passed")
fc.set_indicator_led(0, 255, 0)
sleep(1)
fc.set_indicator_led(0, 0, 0)
sleep(1)

fc.set_flight_mode(fc.PROGRAM_MODE)

target_mission = None
total_mission = 3

logger.info("[MANAGER] Selecting mission...")

if target_mission is None:
    fc.set_indicator_led(0, 0, 255)
    _testing = False
    target_mission = 1
    while True:
        sleep(0.01)
        if fc.event.key_short.is_set():
            fc.event.key_short.clear()
            target_mission = target_mission % total_mission + 1
            for i in range(target_mission):
                fc.set_indicator_led(0, 0, 0)
                sleep(0.1)
                fc.set_indicator_led(0, 0, 255)
                sleep(0.1)
        if fc.event.key_long.is_set():
            fc.event.key_long.clear()
            break
else:
    _testing = True

############################## 开始任务 ##############################
logger.info(f"[MANAGER] Target Mission: {target_mission}")
fc.set_action_log(True)
mission: Any = None
from importlib import import_module

try:
    mis = import_module(f"mission{target_mission}")

    mission = mis.Mission(  # type: ignore
        fc=fc,
        cam=cam,
        rs=rs,
        radar=radar,
        navi=navi,
        mapper=mapper,
        screen=screen,
    )
    logger.info("[MANAGER] Calling Mission")

    mission.run()

    logger.info("[MANAGER] Mission Finished")
except Exception as e:
    logger.exception(f"[MANAGER] Mission Failed")
finally:
    if mission is not None:
        mission.stop()
    if fc.state.unlock.value:
        logger.warning("[MANAGER] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stablize()
        fc.land()
        if not fc.wait_for_lock():
            fc.lock()

############################## 结束任务 ##############################
fc.set_action_log(False)
fc.set_indicator_led(0, 255, 0)
fc.set_digital_output(1, True)
sleep(0.5)
fc.set_digital_output(1, False)
fc.set_indicator_led(0, 0, 0)
sleep(1)
cam.release()
rs.stop()
radar.stop()
fc.close()
RosNodeRunner().stop()
RosManager().kill_all_packages()
########################## 重启自身 #############################
if not _testing:
    self_reboot()
