import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))

from FlightController import FC_Server

fc = FC_Server()
fc.start_listen_serial("COM1", print_state=True)
fc.serve_forever()
