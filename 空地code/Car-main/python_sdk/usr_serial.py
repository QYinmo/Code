import threading
import time
from typing import List

import serial
from loguru import logger
from serial.tools.list_ports import comports
from typing import Callable, Dict, List, Optional


def get_cp2102_com() -> Optional[str]:
    # VID_PID = "66CC:2233"
    # 用于锁定CP2102
    SERIAL_BASE = "SER=0001"
    SERIAL_TYPE = "CP2102"
    # 接入位置锁定: 需要注意, 接入hub后位置会变成 LOCATION=1-3.4 之类的, 其中1-3表示hub在电脑上的USB口位置, 4表示hub上的USB口位置
    POSITION = "LOCATION=1-1.3.2"
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
    POSITION = "LOCATION=1-1.3.1"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if VID_PID in hwid and POSITION in hwid:
            logger.info(f"[FC] Found Arduino hwid on port {port}")
            return port
    return None


def get_wheel_com() -> Optional[str]:
    VID_PID = "1A86:7523"
    # SERIAL_BASE = "SER=0001"
    # SERIAL_TYPE = "CP2102"
    POSITION = "LOCATION=1-2"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if VID_PID in hwid and POSITION in hwid:
            logger.info(f"[FC] Found Arduino hwid on port {port}")
            return port
    return None


def get_radar_com() -> Optional[str]:
    # VID_PID = "66CC:2233"
    # 用于锁定CP2102
    SERIAL_BASE = "SER=0001"
    SERIAL_TYPE = "CP2102"
    # 接入位置锁定: 需要注意, 接入hub后位置会变成 LOCATION=1-3.4 之类的, 其中1-3表示hub在电脑上的USB口位置, 4表示hub上的USB口位置
    POSITION = "LOCATION=1-1.1"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if SERIAL_BASE in hwid and SERIAL_TYPE in desc and POSITION in hwid:
            logger.info(f"[FC] Found Cp2102 hwid on port {port}")
            return port
    return None


class Serial_base(object):
    def __init__(self, device, baudrate, rx_length=3):
        if device == "arduino":
            self.port = get_arduino_com()
        elif device == "cp2102":
            self.port = get_cp2102_com()
        self.ser = serial.Serial(
            port=self.port, baudrate=baudrate, timeout=None)

        self.rx_length = rx_length
        self.send_running = False
        self.listen_running = False
        self.send_count = 3
        self.is_listened = False
        self.port_open()

    def port_open(self):
        self.ser.close()
        if self.ser.is_open is False:
            self.ser.open()

    def send_end(self):
        self.send_running = False

    def send_restart(self):
        self.send_running = True

    def listen_end(self):
        self.listen_running = False

    def listen_restart(self):
        self.listen_running = True

    def independent_send(self, comlist: List[int], count=1):
        for i in range(count):
            for value in comlist:
                hex_value = hex(value)[2:].zfill(2)
                self.ser.write(bytes.fromhex(hex_value))
            time.sleep(0.05)


class Serial_station(Serial_base):

    def send_station(self, comlist: List[int]):
        while self.send_running is True:
            self.ser.reset_output_buffer()
            for value in comlist:
                hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
                self.ser.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口
            time.sleep(0.21)

    def listen_station(self, rxbuffer: List[int]):
        while self.listen_running:
            if self.ser.in_waiting >= self.rx_length:
                frame = self.ser.read(self.rx_length)
                if frame[0] == 0xAA and frame[-1] == 0xFF:
                    rxbuffer[:] = list(frame[1:-1])  # 更新缓冲区
                    self.is_listened = True  # 设置新数据标志

    def send_start(self, comlist: List[int]):
        self.send_running = True
        carsend_thread = threading.Thread(
            target=Serial_station.send_station, args=(self, comlist))
        carsend_thread.daemon = True
        carsend_thread.start()
        logger.info("station发送线程启动")

    def listen_start(self, rxbuffer: List[int]):
        self.listen_running = True
        listen_thread = threading.Thread(
            target=Serial_station.listen_station, args=(self, rxbuffer))
        listen_thread.daemon = True
        listen_thread.start()
        logger.info("station串口监听线程启动")

    def start_transmit(self, comlist: List[int], rxbuffer: List[int]):
        self.listen_start(rxbuffer)
        self.send_start(comlist)


class Serial_gpio(Serial_base):
    def send_gpio(self, comlist: List[int]):
        while self.send_running is True:
            # bytes_in_buffer = self.ser.out_waiting
            # print(f"Bytes in output buffer: {bytes_in_buffer}")
            self.ser.reset_output_buffer()
            for value in comlist:
                hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
                # 将16进制字符串转换为字节并发送到串口
                self.ser.write(bytes.fromhex(hex_value))
                # 查看发送缓存区中的字节数
            time.sleep(0.15)

    def send_start(self, comlist: List[int]):
        self.send_running = True
        gpiosend_thread = threading.Thread(
            target=Serial_gpio.send_gpio, args=(self, comlist))
        gpiosend_thread.daemon = True
        gpiosend_thread.start()
        logger.info("gpio串口发送线程启动")
