from usr_serial import Serial_station
from loguru import logger
import time
terminal_com = [170, 0, 0, 0,0, 255]  # terminal command list
terminal_rxbuffer = [0, 0, 0]  # terminal receive buffer
terminal= Serial_station(device="cp2102", baudrate=115200, rx_length=5)
terminal.send_start(terminal_com)
time.sleep(2)  # 等待串口稳定
logger.info("Terminal串口发送线程启动")
terminal.listen_start(terminal_rxbuffer)
for i in range(1,25):
    terminal_com[2] = i  # 设置目标货物编号
    terminal_com[1] = i  # 设置为定向盘点
    time.sleep(1)  # 等待发送完成
    print(f" {terminal_rxbuffer}")
