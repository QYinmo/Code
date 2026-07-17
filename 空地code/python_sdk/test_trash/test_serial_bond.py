from serial.tools.list_ports import comports
from typing import Callable, Dict, List, Optional
from loguru import logger


def list_all_com() -> List[str]:
    print("Listing all com ports")
    for port, desc, hwid in sorted(comports()):
        print(port, desc, hwid)
    # logger.infor()


def get_fc_com() -> Optional[str]:
    VID_PID = "1A86:7523"
    # SERIAL_BASE = "SER=0001"
    # SERIAL_TYPE = "CP2102"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if VID_PID in hwid:
            logger.info(f"[FC] Found Arduino hwid on port {port}")
            return port
    return None


def get_radar_com() -> Optional[str]:
    VID_PID = "1A86:7523"
    # SERIAL_BASE = "SER=0001"
    # SERIAL_TYPE = "CP2102"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if VID_PID in hwid:
            logger.info(f"[FC] Found Arduino hwid on port {port}")
            return port
    return None

def get_cp2102_com() -> Optional[str]:
    # VID_PID = "66CC:2233"
    ## 用于锁定CP2102
    SERIAL_BASE = "SER=0001"
    SERIAL_TYPE = "CP2102"
    # 接入位置锁定: 需要注意, 接入hub后位置会变成 LOCATION=1-3.4 之类的, 其中1-3表示hub在电脑上的USB口位置, 4表示hub上的USB口位置
    POSITION = "LOCATION=1-3.4"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if SERIAL_BASE in hwid and SERIAL_TYPE in desc and POSITION in hwid:
            logger.info(f"[FC] Found Cp2102 hwid on port {port}")
            return port
    return None

def get_arduino_com() -> Optional[str]:
    VID_PID = "1A86:7523"
    # SERIAL_BASE = "SER=0001"
    # SERIAL_TYPE = "CP2102"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if VID_PID in hwid:
            logger.info(f"[FC] Found Arduino hwid on port {port}")
            return port
    return None


#list_all_com()
print(get_cp2102_com())
print(get_arduino_com())