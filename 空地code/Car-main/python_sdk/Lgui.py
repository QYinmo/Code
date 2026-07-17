import tkinter as tk
from PIL import Image, ImageTk
import time
import numpy as np
import cv2
# 画图
# 创建窗口对象


class tk_gui:
    root = tk.Tk()
    # 获取屏幕宽度和高度
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    # 设置窗口大小和位置
    root.geometry("%dx%d+0+0" % (screen_width, screen_height))
    # 设置窗口题
    root.title("Fullscreen Borderless Window")
    # 隐藏窗口的边框和菜单栏
    root.overrideredirect(1)
    label = tk.Label(root)
    label.pack()

    def __init__(self):
        self.button_font = ("Arial", 12)
        self.label_font = ("Arial", 12)
        self.button_width = 20
        self.button_height = 8
        self.task_id = -1
        self.ready_to_go = False

    def button_clicked(self, button_id):
        if 1 <= button_id <= 2:
            self.task_id = button_id
        elif button_id == 3:
            if self.task_id == -1:
                self.output_text.configure(text="Please select a task")
                return
            else:
                self.ready_to_go = True
        text = f"Current task: {self.task_id} Ready to start: {self.ready_to_go}"
        self.output_text.configure(text=text)

    def gui_window(self):
        self.root.title("UAV Control GUI")
        for i in range(2):
            button = tk.Button(
                self.root, text=f"Task {i+1}", command=lambda button_id=i+1: self.button_clicked(button_id))
            button.config(font=self.button_font,
                          width=self.button_width, height=self.button_height)
            button.place(relx=1 - i * 0.1, rely=0.5, anchor='e')  # 按钮从右侧中部向左分布

        button = tk.Button(
            self.root, text="Start", command=lambda button_id=3: self.button_clicked(button_id))
        button.config(font=self.button_font,
                      width=self.button_width, height=self.button_height)
        button.place(relx=0.8, rely=0.5, anchor='e')

        # 创建文本输出框
        self.output_text = tk.Label(
            self.root, text="", font=self.label_font, anchor=tk.CENTER)
        self.output_text.place(
            relx=0.5, rely=1.0, anchor='s', y=-10)  # 位于屏幕正下方
        text = "No task selected"
        self.output_text.configure(text=text)

        self.output_text0 = tk.Label(
            self.root, text="", font=self.label_font, anchor=tk.CENTER)
        self.output_text0.place(
            relx=0.5, rely=1.0, anchor='s', y=-40)  # 位于output_text上方
        self.output_text0.configure(
            text='Map scale: 10cm/small grid 80cm/large grid')

        self.output_text1 = tk.Label(
            self.root, text="Total Distance: 0", font=self.label_font, anchor=tk.CENTER)
        self.output_text1.place(
            relx=0.5, rely=1.0, anchor='s', y=-70)  # 位于output_text0上方

        self.output_text2 = tk.Label(
            self.root, text="Fire source location: Not found", font=self.label_font, anchor=tk.CENTER)
        self.output_text2.place(
            relx=0.5, rely=1.0, anchor='s', y=-100)  # 位于output_text2上方

        self.exit_button = tk.Button(
            self.root, text="Exit", command=self.root.quit, width=8, height=3)
        self.exit_button.place(
            relx=1.0, rely=0.0, anchor='ne', x=-10, y=10)  # 退出按钮

    def show_img(self, img):
        img1 = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        photo = ImageTk.PhotoImage(image=Image.fromarray(img1))
        self.label.config(image=photo)
        self.label.image = photo
        self.label.place(x=0, y=0)

    def start(self):
        self.gui_window()
        self.root.mainloop()


class cv_draw:
    def __init__(self):
        self.width = 1200
        self.height = 1200

    def draw(self, points):
        image = cv2.imread('/home/pi/workspace/Car/python_sdk/base.png')
        for i in range(len(points)):
            x = -points[i][1] + 35
            y = -points[i][0] + 365
            cv2.circle(image, (x, y), 2, (0, 0, 255), -1)
        return image

    def show_draw_res(self, arr):
        res = self.draw(arr)
        return res


if __name__ == "__main__":
    gui = tk_gui()
    gui.start()
