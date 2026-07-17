from FlightController.Components import LD_Radar
from loguru import logger

radar = LD_Radar()
radar.start(radar_type= "LD06")

#radar.register_map_func(radar.map.find_nearest_with_ext_point_opt, from_=45, to_=135, num=3)
radar.show_radar_map()
