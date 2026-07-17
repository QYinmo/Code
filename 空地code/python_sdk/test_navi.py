from usr_serial import Serial_base, Serial_station
import time
from loguru import logger
serial_terminal = Serial_station(
                device="cp2102", baudrate=9600, rx_length=3)
terminal_com =   [170, 0, 0, 0, 0, 0, 0, 0, 0,0,0,0 ,0,255]

serial_terminal.send_start(terminal_com)
logger.info("[MISSION] Start ...")
for i in range(200):
 terminal_com[1] = i
 time.sleep(0.5)
