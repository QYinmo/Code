from time import sleep
import serial

ser = serial.Serial(port="/dev/ttyAMA0", baudrate=115200)
rxbuffer = []

while True:
    # print(my_serial.read(20))
    byte_data = ser.read() 
    if byte_data == b'\xAA':
        # 读取接下来的四个字节数据
        recv = ser.read(9)
        # 判断数据是否符合通信协议，即以0xFF结尾
        if recv[-1] == 0xFF:
            rxbuffer.clear()
            for i in range(0, 8):
                rxbuffer.append(recv[i])
        print(recv[:-1])
    sleep(0.01)
# sleep(0


