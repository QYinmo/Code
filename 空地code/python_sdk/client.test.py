import time

from FlightController import FC_Client, FC_Controller, FC_Server

fc = FC_Controller()
fc.start_listen_serial(print_state=True)

while True:
    time.sleep(1)
