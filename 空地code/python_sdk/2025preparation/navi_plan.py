
"""
self.next_point_event = threading.Event()


self.next_point_event.set()


for self.location in range(len(MISSION_POINTS)):
            navi.navigation_to_waypoint(MISSION_POINTS[self.location ])
            self.navi.wait_for_waypoint()
            logger.info(f"[MISSION] Go to target point {MISSION_POINTS[self.location ]}")
            self.next_point_event.clear()
            self.identify_status = True
            self.next_point_event.wait()
            self.next_point_event.clear()
            fc.set_indicator_led(0,0,255) # 蓝灯提示
            time.sleep(0.5)
            fc.set_indicator_led(0,0,0)
        navi.pointing_landing(LANDING_POINT)
定点到点识别
"""

