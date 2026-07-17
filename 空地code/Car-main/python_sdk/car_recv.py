import serial
import time
import numpy as np
import cv2
from Lgui import tk_gui, cv_draw
import tkinter as tk
import threading
import os
PATH = os.path.dirname(os.path.abspath(__file__))
class CommunicationApp:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        self.ser = serial.Serial(port, baudrate)

        print("串口通信启动")
        self.received_data = [0, 0, 0, 0]
        self.com_list = [170, 0, 0, 255]
        self.gui = tk_gui()
        self.draw = cv_draw()
        self.totaldis = 0
        self.lastxy = [0, 0]  # 最后一次送过来的点
        self.timecount = 0
        self.firepoint = [0,0]
        self.fire_go = False
        self.lastdata = [0, 0]  # 上一次GUI更新打的点 1s更新一次
        self.datalist = [[0, 0]]  # 每次gui更新间隔中收到的点
        self.recv_bias = 5

    def data_receive_task(self, rxbuffer):
        while True:
            byte_data = self.ser.read()
            if byte_data == b'\xAA':
                recv = self.ser.read(5)
                if recv[-1] == 0xFF:  # 火源位置两位 飞机位置两位
                    rxbuffer.clear()
                    rxbuffer.append(recv[0])
                    rxbuffer.append(recv[1])
                    rxbuffer.append(recv[2])
                    rxbuffer.append(recv[3])
                    if rxbuffer[2] != 0 and rxbuffer[3] != 0:
                        self.firepoint = [
                            rxbuffer[2]*self.recv_bias/100, rxbuffer[3]*self.recv_bias/100]
                    self.datalist.append(
                        [rxbuffer[0]*self.recv_bias, -rxbuffer[1]*self.recv_bias])
                    self.lastxy = [rxbuffer[0] *
                                   self.recv_bias, -rxbuffer[1]*self.recv_bias]
                    print(recv)
                    #print(self.lastxy)
            time.sleep(0.1)

    def data_send_task(self, comlist):
        while True:
            for value in comlist:
                hex_value = hex(int(value))[2:].zfill(2)  # 将数组中的每个值转换成16进制字符串
                self.ser.write(bytes.fromhex(hex_value))  # 将16进制字符串转换为字节并发送到串口
            time.sleep(0.05)

    def update_gui(self):
        if time.time() - self.timecount > 1:
            distance = int(((abs(self.lastdata[0]) - abs(self.lastxy[0])) ** 2 + (
                abs(self.lastdata[1]) - abs(self.lastxy[1])) ** 2) ** 0.5)
            self.totaldis += distance
            self.lastxy = self.lastdata
            self.gui.output_text1.configure(text=f"Total Distance:{self.totaldis}")
            if self.firepoint != [0, 0]:
                self.gui.output_text2.configure(text=f"Fire source location:{self.firepoint[0]*100},{self.firepoint[1]*100}")
            img = self.draw.show_draw_res(self.datalist)
            self.gui.show_img(img)
            self.timecount = time.time()

    def handle_gui(self):
        while True:
            if self.gui.task_id != -1:
                self.com_list[2] = self.gui.task_id
                #print("task_id:", self.gui.task_id)
                if self.gui.ready_to_go is True:
                    self.com_list[1] = 1
                    #print("飞机go")
            if self.firepoint[0] != 0 and self.firepoint[1] != 0 and self.fire_go is False:
                print("fire_go")
                os.system(f"python3 {PATH}/run.py -x {self.firepoint[1]} -y {self.firepoint[0]} &")
                self.fire_go = True
            time.sleep(0.1)

    def main_loop(self):
        self.update_gui()
        self.gui.root.after(200, self.main_loop)

    def run(self):
        threading.Thread(target=self.data_receive_task,
                         args=(self.received_data,), daemon=True).start()
        threading.Thread(target=self.data_send_task, args=(
            self.com_list,), daemon=True).start()
        threading.Thread(target=self.handle_gui, daemon=True).start()
        self.gui.root.after(50, self.main_loop)
        self.gui.gui_window()
        self.gui.root.mainloop()


if __name__ == "__main__":
    app = CommunicationApp("/dev/ttyUSB1")
    app.run()
