import math
import time

from FlightController import FC_Client, FC_Controller


def hsv_to_rgb_255(h, s, v):
    h = h / 360
    s = s / 100
    v = v / 100
    if s == 0.0:
        v *= 255
        return v, v, v
    i = int(h * 6.0)
    f = (h * 6.0) - i
    p = int(255 * v * (1.0 - s))
    q = int(255 * v * (1.0 - (s * f)))
    t = int(255 * v * (1.0 - (s * (1.0 - f))))
    v = int(v * 255)
    i %= 6
    if i == 0:
        return v, t, p
    if i == 1:
        return q, v, p
    if i == 2:
        return p, v, t
    if i == 3:
        return p, q, v
    if i == 4:
        return t, p, v
    if i == 5:
        return v, p, q


# fc = FC_Client()
# fc.connect("fc-pc")
fc = FC_Controller()
fc.start_listen_serial()
fc.wait_for_connection()
fc.set_rgb_led(0, 0, 0)
fc.set_indicator_led(0, 0, 0)
fc.set_action_log(False)
time.sleep(1)
while True:
    for i in range(0, 360, 10):
        time.sleep(0.01)
        rgb = hsv_to_rgb_255(i, 100, 100)
        fc.set_rgb_led(rgb[0], rgb[1], rgb[2])
        # fc.set_indicator_led(rgb[0], rgb[1], rgb[2])
