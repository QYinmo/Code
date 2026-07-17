import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

from FlightController import FC_Server

fc = FC_Server()
fc.start_listen_serial(print_state=False, block_until_connected=True)
fc.serve_forever(indicator=True)
