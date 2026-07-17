from usr_serial_fc import Serial_station
import time


terminal_com=[170,0,0,0,0,0,0,0,255]
terminal_rxbuffer=[]
serial_terminal = Serial_station(
                device="cp2102", baudrate=115200)
serial_terminal.start_transmit(terminal_com,terminal_rxbuffer)
while True:
        if terminal_rxbuffer:
          print(f"{terminal_rxbuffer}")
        time.sleep(0.2)