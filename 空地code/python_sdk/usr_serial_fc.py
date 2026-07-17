import threading
import time
from typing import List

import serial
from loguru import logger
from serial.tools.list_ports import comports
from typing import Callable, Dict, List, Optional


def get_cp2102_com() -> Optional[str]:
    VID_PID = "10C4:EA60"
    # 用于锁定CP2102
    SERIAL_BASE = "SER=0001"
    SERIAL_TYPE = "CP2102"
    # 接入位置锁定: 需要注意, 接入hub后位置会变成 LOCATION=1-3.4 之类的, 其中1-3表示hub在电脑上的USB口位置, 4表示hub上的USB口位置
    POSITION = "LOCATION=3-5"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if SERIAL_BASE in hwid and SERIAL_TYPE in desc and POSITION in hwid:
            logger.info(f"[FC] Found Cp2102 hwid on port {port}")
            return port
    return None


def get_arduino_com() -> Optional[str]:
    VID_PID = "0403:6001"
    # SERIAL_BASE = "SER=0001"
    # SERIAL_TYPE = "CP2102"
    for port, desc, hwid in sorted(comports()):
        print(desc, hwid)
        if VID_PID in hwid:
            logger.info(f"[FC] Found Arduino hwid on port {port}")
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
        self.listen_running = False
        self.is_listened = False
        self.port_open()

    def port_open(self):
        self.ser.close()
        if self.ser.is_open is False:
            self.ser.open()
    def listen_end(self):
        self.listen_running = False

    def listen_restart(self):
        self.listen_running = True
        
class Serial_station(Serial_base):
    def __init__(self, device, baudrate):
        super().__init__(device, baudrate)
        self.frame_start = 0xAA
        self.frame_end = 0xFF

    def send_station(self, comlist: List[int]):
        while self.send_running:
            self.ser.reset_output_buffer()
            # 添加帧头和帧尾
            data_to_send = [self.frame_start] + comlist + [self.frame_end]
            for value in data_to_send:
                hex_value = hex(value)[2:].zfill(2)
                self.ser.write(bytes.fromhex(hex_value))
            time.sleep(0.15)

    def send_start(self, comlist: List[int]):
        self.send_running = True
        threading.Thread(
            target=self.send_station, 
            args=(comlist,),
            daemon=True
        ).start()
        logger.info("station串口发送线程启动")

    def listen_station(self, rxbuffer: List[int]):
        buffer = []
        in_frame = False
        
        while self.listen_running:
            try:
                if self.ser.in_waiting > 0:
                    byte = self.ser.read(1)[0]
                    
                    if byte == self.frame_start:
                        in_frame = True
                        buffer = []
                    elif byte == self.frame_end and in_frame:
                        in_frame = False
                        rxbuffer[:] = buffer
                        self.is_listened = True
            
                    elif in_frame:
                        buffer.append(byte)
                        
            except Exception as e:
                logger.error(f"接收数据错误: {e}")
                time.sleep(0.1)

    def listen_start(self, rxbuffer: List[int]):
        self.listen_running = True
        threading.Thread(
            target=self.listen_station, 
            args=(rxbuffer,),
            daemon=True
        ).start()
        logger.info("station串口监听线程启动")
    def listen_end(self):
        self.listen_running = False
    def start_transmit(self, comlist: List[int], rxbuffer: List[int]):
        self.send_start(comlist)
        self.listen_start(rxbuffer)
        time.sleep(0.1)

class Serial_gpio(Serial_base):
    def send_gpio(self, comlist: List[int]):
        while self.send_running is True:
            #bytes_in_buffer = self.ser.out_waiting
            #print(f"Bytes in output buffer: {bytes_in_buffer}")  
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
    def close(self):

        if self.ser.is_open:
            self.ser.close()
            logger.info("串口已关闭")
        else:
            logger.warning("串口未打开，无法关闭")
        self.listen_end()
        self.send_running = False