import os
import time
from time import sleep
os.chdir(os.path.dirname(os.path.abspath(__file__)))

x = 6
y = 6

import numpy as np
# from fire_vision import Detector
from FlightController import FC_Client, FC_Controller
from FlightController.Components import LD_Radar
from loguru import logger
from SC import Controller, State

speed = 0.0

fc = FC_Client()


def main():
    fc.connect()
    fc.wait_for_connection()
    for i in range(10):
        print(i)
        fc.set_steer_and_speed(4, 0)
        sleep(1)
        fc.set_steer_and_speed(4, 2)
        sleep(1)
        fc.set_steer_and_speed(4, -2)
        sleep(1)

main()