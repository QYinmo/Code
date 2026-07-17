import math
import time

from FlightController import FC_Controller
from FlightController.Components.RealSense import T265
from FlightController.Components.RosNode import RosNodeRunner
from loguru import logger

fc = FC_Controller()
fc.start_listen_serial()
t265 = T265("ros")
t265.start(event_skip=4, print_update=True)
runner = RosNodeRunner().add_nodes().run()

try:
    while True:
        time.sleep(1)
finally:
    t265.stop()
    # runner.stop()
