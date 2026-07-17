import math
import time

from FlightController import FC_Controller
from FlightController.Components.RosNode import RosNodeRunner
from FlightController.Components.RosMapper import RosMapper
from loguru import logger

fc = FC_Controller()
fc.start_listen_serial()
mapper = RosMapper()
RosNodeRunner().add_nodes().run()

while True:
    # if mapper.map_update_event.wait(0.01):
    #     mapper.map_update_event.clear()
    #     logger.debug(f"map: {mapper.map.info.origin}")
    if mapper.trans_update_event.wait(0.01):
        mapper.trans_update_event.clear()
        # logger.debug(f"trans: {mapper.trans}")
        print(mapper.position,mapper.eular_rotation)
