from FlightController import FC_Server
import os
from usr_serial import get_wheel_com
os.chdir(os.path.dirname(os.path.abspath(__file__)))

port = get_wheel_com()
fc = FC_Server()
fc.start_listen_serial(port, print_state=True,
                       block_until_connected=True)
fc.serve_forever(port=2333)
