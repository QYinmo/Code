import serial
import time
import os
import time
import numpy as np
from FlightController import FC_Client, FC_Controller, FC_Like
from FlightController.Components import LD_Radar
from loguru import logger
import cv2
import threading
from serial.tools.list_ports import comports
from Vision_plus import debug_imshow, HighPrecisionFPS, get_ROI, find_anycolor_area, vision_debug, open_camera_plus, set_manual_exporsure, find_red_area
from usr_serial import Serial_gpio, Serial_station
from simple_pid import PID
PATH = os.path.dirname(os.path.abspath(__file__))
timer = HighPrecisionFPS()

SERIAL_PORT = 'COM17'
BAUD_RATE = 115200      # 波特率，必须与单片机代码中设置的波特率一致
TIMEOUT = 1           # 读取串口数据的超时时间 (秒)。如果设置为 None，则会一直等待直到读到数据。
gpio_com = [170, 0, 0, 0, 0, 255]
x_pid = PID(Kp=0.02, Ki=0.00, Kd=0.0, setpoint=0)
y_pid = PID(Kp=0.02, Ki=0.0000, Kd=0.0, setpoint=0)


def start_ser():
    threading.Thread(target=serial_send_thread_func,
                     daemon=True).start()


def serial_send_thread_func():
    global gpio_com  # 声明要使用全局变量

    try:
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print(f"成功打开串口: {SERIAL_PORT}，波特率: {BAUD_RATE}")
            ser.reset_output_buffer()

            while True:  # 持续循环发送
                for value in gpio_com:
                    hex_value = hex(value)[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
                    ser.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口

                time.sleep(0.22)


    except serial.SerialException as e:
        logger.error(f"串口错误线程终止: {e}")
    except Exception as e:
        logger.error(f"串口发送线程发生未知错误并终止: {e}")


def servo1(state):  # 下面电机
    if state!= 0:
        
        gpio_com[1] = abs(10*int(state))
        # 避免除以0，如果state是0，方向设为0
        gpio_com[2] = (int(state / abs(state)) if state != 0 else 0)+1
    else:
        gpio_com[1] = 0
        gpio_com[2] = 1

def servo2(state):  # 上面电机
    if state!= 0:
        gpio_com[3] = abs(10*round(state))
        if state>0:
            gpio_com[4] = 2
        else :
            gpio_com[4] = 1
    else:
        gpio_com[3] = 0
        gpio_com[4] = 1 

def id_function(img):

    debug_imshow(img, "Origin")
    logger.debug("正在识别")
    f, dx, dy, area = find_red_area(img, 500)
    if f:
        logger.info(f"area:{area}")
        return True, dx, dy, area
    return False, 0, 0, 0


def target(cam):
    logger.info("找到东西，开始瞄准")
    # area_pre = 100
    while True:
        ret, img = cam.read()
        if not ret:
            logger.warning("没有图像")
            servo1(0)
            servo2(0)
            continue

        # img = get_ROI(img, (0.27, 0.27, 0.46, 0.46))
        f, dx, dy, _ = id_function(img)
        if f:
            logger.info("找到了")
            x_control = x_pid(dx)
            y_control = y_pid(dy)
            logger.debug(f"x_control:{x_control},y_control{y_control}")
            logger.debug(f"x:{dx},y{dy}")
            servo1(x_control)
            servo2(y_control)
        else:
            servo1(0)
            servo2(0)
            logger.info("无")
        time.sleep(0.18)


if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # 0表示默认摄像头
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
    cap.set(cv2.CAP_PROP_EXPOSURE, -10)
    vision_debug()
    start_ser()
    target(cap)
    cap.release()
    cv2.destroyAllWindows()
