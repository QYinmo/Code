import struct
import time
from typing import Optional, Tuple, Union
import numpy as np

from FlightController.Base import Byte_Var, FC_Base_Uart_Comunication
from loguru import logger


class FC_Protocol(FC_Base_Uart_Comunication):
    """
    协议层, 定义了实际的控制命令
    """

    # constants
    MOTOR_L = 0x01
    MOTOR_R = 0x02
    BREAK = 0x00
    GLIDE = 0x01
    SPD_CTRL = 0x02
    POS_CTRL = 0x03
    MANUAL = 0x04

    SERVO1 = 0x01
    SERVO2 = 0x02
    SERVO3 = 0x04
    SERVO4 = 0x08

    # 控制帧格式: [0xAA] [option] [data_vx] [data_vz_sign] [data_vz] [checksum] [0xFF]

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)  # type: ignore
        self._byte_temp1 = Byte_Var()
        self._byte_temp2 = Byte_Var()
        self._byte_temp3 = Byte_Var()
        self._byte_temp4 = Byte_Var()

    def _action_log(self, action: str, data_info: Optional[str] = None):
        if self.settings.action_log_output:
            string = f"[FC] [ACTION] {action.upper()}"
            if data_info is not None:
                string += f" -> {data_info}"
            logger.info(string)

    def send_to_wireless(self, data: bytes) -> None:
        """
        转发数据到无线串口
        """
        self.send_data_to_fc(data, 0x02)

    def set_steer_and_speed(self, v_x: int, v_z: float):
        """
        设置转向舵机和后轮速度
        """
        v_z = max(-5, min(v_z, 5))
        v_x = max(0, min(v_x, 1500))
        flag = 0
        if v_z < 0:
            flag = 1
        v_z = 50.0*v_z
        v_z = int(v_z)
        v_x = int(v_x*0.4)
        logger.info(f"v_x{v_x}v_z{v_z}")
        self.send_data_to_fc(struct.pack(
            "<BBB", v_x, flag, abs(v_z)), 0x01, False, True)
        packed_data = struct.pack("<BBB", v_x, flag, abs(v_z))
        print(f"Packed Data (bytes object): {packed_data}")
        print(f"Packed Data (hex representation): {packed_data.hex()}")
