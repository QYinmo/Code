import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import time
import queue
from loguru import logger
import random
from typing import List
from usr_serial import Serial_station, Serial_gpio
class GroundStation:
    def __init__(self, root):

        self.root = root
        self.root.attributes('-fullscreen', True)  
        self.root.bind('<Escape>', lambda e: self.root.attributes('-fullscreen', False))  # 按ESC退出全屏
        self.root.title("立体货架盘点无人机系统 - 地面站 ")
        self.root.geometry("1200x800")
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.last_pos = 0
        self.data_queue = queue.Queue()
        self.usr_serial=True
        if self.usr_serial:
            self.serial_terminal = Serial_station(device="cp2102", baudrate=115200, rx_length=6)
            self.serial_gpio = Serial_gpio(device="arduino", baudrate=38400)
        self.terminal_com = [170, 0, 0, 0, 255]  # 起飞信号，任务编号，目标货物位置
        self.terminal_rxbuffer = [0,0,0,0]#货物pos，货物id，定向货物id,是否完成任务2
        self.serial_terminal.start_transmit(self.terminal_com, self.terminal_rxbuffer)
        self.gpio_com = [170, 0, 255]  # LED
        self.serial_gpio.send_start(self.gpio_com)
        # 盘点结果存储
        self.inventory_data = {}
        self.target_id = None

        # 初始化盘点数据
        self.initialize_inventory_data()

        # 创建UI
        self.setup_ui()

        self.root.after(100, self.update_ui)

    def led(self):
        self.gpio_com[1] = 1 
        logger.info("LED已开启")
        self.root.after(1000, lambda: setattr(self, 'gpio_com', [170, 0, 255]))

    def initialize_inventory_data(self):
        """初始化盘点数据，创建A1-D6共24个位置"""
        positions = []
        for face in ['A', 'B', 'C', 'D']:
            for i in range(1, 7):
                positions.append(f"{face}{i}")

        for pos in positions:
            self.inventory_data[pos] = {"id": None, "scanned": False}

    def setup_ui(self):
        """设置用户界面"""
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 左侧：控制面板和状态
        control_panel_frame = ttk.LabelFrame(main_frame, text="控制与状态 ", padding="10")
        control_panel_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

        # 右侧：盘点结果和地图显示
        display_panel_frame = ttk.LabelFrame(main_frame, text="盘点信息与航线", padding="10")
        display_panel_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)

        # --- 左侧控制面板布局 ---
        # 系统状态
        status_frame = ttk.LabelFrame(control_panel_frame, text="系统状态", padding="5")
        status_frame.pack(fill=tk.X, pady=5)

        self.status_label = ttk.Label(status_frame, text="运行中", foreground="blue")
        self.status_label.pack(pady=2)

        self.status_led = tk.Canvas(status_frame, width=30, height=30, bg="green")
        self.status_led.pack(pady=5)

        # 任务控制
        task_control_frame = ttk.LabelFrame(control_panel_frame, text="任务控制", padding="5")
        task_control_frame.pack(fill=tk.X, pady=5)

        ttk.Label(task_control_frame, text="任务类型:").pack(anchor=tk.W, pady=2)
        self.task_var = tk.StringVar(value="full")
        ttk.Radiobutton(task_control_frame, text="全盘盘点", variable=self.task_var, value="full").pack(anchor=tk.W, padx=20, pady=25)
        ttk.Radiobutton(task_control_frame, text="定向盘点", variable=self.task_var, value="target").pack(anchor=tk.W, padx=20, pady=25)

        ttk.Label(task_control_frame, text="目标货物编号 (定向盘点):").pack(anchor=tk.W, pady=2)
        self.target_entry = ttk.Entry(task_control_frame)
        self.target_entry.pack(fill=tk.X, pady=2)
        self.target_entry.insert(0, "")

        start_btn = ttk.Button(task_control_frame, text="开始任务", command=self.start_task)
        start_btn.pack(fill=tk.X, padx=20,pady=25)

        # --- 右侧显示面板布局 ---
        # 盘点结果文字输出区域
        self.inventory_text_display = tk.Text(display_panel_frame, wrap=tk.WORD, height=30)  # 24行足够显示所有24个货物
        self.inventory_text_display.pack(fill=tk.X, pady=5)
        self.inventory_text_display.config(state=tk.DISABLED)

        search_frame = ttk.Frame(display_panel_frame)
        search_frame.pack(fill=tk.X, pady=5)

        ttk.Label(search_frame, text="搜索货物编号:").pack(side=tk.LEFT)
        self.search_entry = ttk.Entry(search_frame)
        self.search_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        search_btn = ttk.Button(search_frame, text="查询", command=self.search_item)
        search_btn.pack(side=tk.LEFT)

        self.search_result_label = ttk.Label(display_panel_frame, text="查询结果: 无")
        self.search_result_label.pack(fill=tk.X, pady=8)

        # 路径规划显示 - 调整高度以适应新的布局
        self.path_canvas = tk.Canvas(display_panel_frame, width=500, height=300, bg="white", bd=2, relief="sunken")
        self.path_canvas.pack(fill=tk.BOTH, expand=True, pady=5)
        self.draw_warehouse_layout()
        # --- 右上角退出按钮 ---
        exit_button = ttk.Button(
            self.root, 
            text="退出", 
            command=self.on_closing,
            width=20,  # 宽度（字符单位）
        )

        exit_button.place(relx=1.0, rely=0.0, anchor='ne', x=-10, y=10)


    def update_ui(self):
        """定时从队列中获取数据并更新UI"""
        if self.terminal_rxbuffer[3] == 1:
            self.led()
        logger.debug(f"接收到数据1: {self.terminal_rxbuffer}")
        if self.serial_terminal.is_listened:
            logger.debug(f"接收到数据2: {self.terminal_rxbuffer}")
            id = self.terminal_rxbuffer[1]
            pos = self.terminal_rxbuffer[0]
            if pos != self.last_pos and id != 0:
                self.last_pos = pos
                self._flash_led()
                if pos in range(1,7):
                    pos_key = f"A{pos}"
                elif pos in range(7,13):
                    pos_key = f"B{pos-6}"
                elif pos in range(13,19):
                    pos_key = f"C{pos-12}"
                elif pos in range(19,25):
                    pos_key = f"D{pos-18}"
                self.inventory_data[pos_key]["id"] = id
                self.inventory_data[pos_key]["scanned"] = True
                self.led()
                self.update_inventory_display()
        
        self.root.after(100, self.update_ui) 

    def _flash_led(self):
        """模拟LED闪烁"""
        self.status_led.config(bg="yellow")
        self.root.after(200, lambda: self.status_led.config(bg="green"))

    def update_inventory_display(self):
        """更新盘点结果文字输出区域"""
        self.inventory_text_display.config(state=tk.NORMAL)
        self.inventory_text_display.delete(1.0, tk.END)

        sorted_positions = sorted(self.inventory_data.keys(), key=lambda x: (x[0], int(x[1:])))

        display_text = "--- 全盘点结果 ---\n\n"
        scanned_count = 0
        for pos in sorted_positions:
            data = self.inventory_data[pos]
            item_id = data["id"] if data["id"] is not None else "未扫描"
            status = "已扫描" if data["scanned"] else "未扫描"
            
            display_text += f"位置: {pos}, 货物编号: {item_id}, 状态: {status}\n"
            if data["scanned"]:
                scanned_count += 1
        
        display_text += f"\n总计已扫描货物: {scanned_count} / {len(self.inventory_data)}\n"
        
        self.inventory_text_display.insert(tk.END, display_text)
        self.inventory_text_display.config(state=tk.DISABLED)

    def draw_warehouse_layout(self):
        """绘制仓库布局"""
        canvas = self.path_canvas
        canvas.delete("all")

        # 仓库边界
        canvas.create_rectangle(10, 10, 460, 360, outline="black", width=2)

        # 起飞点 (黑色方形)
        canvas.create_rectangle(60, 260, 110, 310, fill="black")
        canvas.create_text(85, 285, text="起飞点", fill="white")

        # 降落点 (黑色圆形)
        canvas.create_oval(385, 35, 435, 85, fill="black")
        canvas.create_text(410, 60, text="降落点", fill="white")

        # 货架A (左侧)
        self.draw_shelf(canvas, 160, 260, "A")


        self.draw_shelf(canvas, 330, 260, "C")


    def draw_shelf(self, canvas, x, y, face):
        """绘制单个货架""" 
        # 货架板面 (白色矩形)
        canvas.create_rectangle(x, y, x+5, y-200, fill="white", outline="black")
        
        # 货架标识
        canvas.create_text(x-25, y-200, text=f"货架{face}面", font=("Arial", 10, "bold"))
        if face == "A":
            canvas.create_text(x+30, y-200, text="货架B面", font=("Arial", 10, "bold"))
        elif face == "C":
            canvas.create_text(x+30, y-200, text="货架D面", font=("Arial", 10, "bold"))        
    


    def find_position_by_id(self, item_id):
        """根据货物编号查找位置"""
        for pos, data in self.inventory_data.items():
            if data["id"] == item_id:
                return pos
        return None

    def start_task(self):
        """开始任务"""
        task_type = self.task_var.get()
        
        if task_type == "full":
            self.terminal_com[1] = 1
            self.terminal_com[2] = 0
            self.root.after(1000, lambda: setattr(self, 'terminal_com', [170,0,0, 0, 255]))
            self.status_label.config(text="开始全盘盘点任务 (模拟)", foreground="blue")
            self._log_message("模拟: 开始全盘盘点任务")
            
            # 重置盘点状态
            for pos in self.inventory_data:
                self.inventory_data[pos]["scanned"] = False
                self.inventory_data[pos]["id"] = None
            
            self.update_inventory_display()
        
        elif task_type == "target":
            try:
                self.terminal_com[2] = 1  # 设置为定向盘点
                if self.terminal_rxbuffer[2] != 0 and self.terminal_rxbuffer[2] in range(1, 25):
                    self.target_id = self.terminal_rxbuffer[2]
                    self.terminal_com[1] = 1  
                      # 设置为定向盘点

                    if not (1 <= self.target_id <= 24):
                        raise ValueError("货物编号必须在1-24之间")

                    for i in range(1, 7):
                        if self.inventory_data[f"A{i}"]["id"] == self.target_id:
                            self.target_pos = i
                            break
                        elif self.inventory_data[f"B{i}"]["id"] == self.target_id:
                            self.target_pos = 6+i
                            break
                        elif self.inventory_data[f"C{i}"]["id"] == self.target_id:
                            self.target_pos = 12+i
                            break
                        elif self.inventory_data[f"D{i}"]["id"] == self.target_id:
                            self.target_pos = 18+i
                            break

                    self.status_label.config(text=f"开始定向盘点任务 , 目标: {self.target_pos}", foreground="blue")
                    self._log_message(f" 开始定向盘点任务, 目标ID {self.target_pos}")
                    self.terminal_com[3] = self.target_pos  # 设置目标货物编号
                    # 绘制路径
                    self.draw_target_path_approx(self.target_pos)

            except ValueError as e:
                self.status_label.config(text=f"错误: {e}", foreground="red")
                self._log_message(f"开始任务失败: {e}")



    def draw_target_path_approx(self, target_id):
        """绘制定向盘点路径"""
        canvas = self.path_canvas
        canvas.delete("path")
        # 根据目标ID确定大致位置
        if target_id%3==1:
           target_y=110
        elif target_id%3==2:
           target_y=160
        elif target_id%3==0:
           target_y=210

        if target_id <= 6:  # A面
            target_x = 110
        elif target_id <= 12:  # B面
            target_x = 210
        elif target_id <= 18:  # C面
            target_x = 280
        else:  # D面
            target_x = 380
        
        # 绘制路径: 起飞点 -> 目标附近 -> 降落点
        canvas.create_line(85, 285, target_x, 285, fill="red", width=2, arrow=tk.LAST, tags="path")
        canvas.create_line(target_x, 285, target_x, target_y, fill="red", width=2, arrow=tk.LAST, tags="path")




    def search_item(self):
        """搜索货物"""
        query = self.search_entry.get().strip()
        
        if not query:
            self.search_result_label.config(text="请输入货物编号")
            return
        
        try:
            query_id = int(query)
            if not (1 <= query_id <= 24):
                self.search_result_label.config(text="货物编号必须在1-24之间")
                return
            
            found_pos = None
            for pos, data in self.inventory_data.items():
                if data["id"] == query_id:
                    found_pos = pos
                    break
            
            if found_pos:
                self.search_result_label.config(text=f"货物 {query_id} 位于 {found_pos}")
            else:
                self.search_result_label.config(text=f"未找到货物 {query_id}")
        
        except ValueError:
            self.search_result_label.config(text="请输入有效数字")

    def _log_message(self, message):
        """记录日志消息"""
        print(f" {message}")

    def on_closing(self):
        """安全关闭窗口"""
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = GroundStation(root)
    root.mainloop()


                         