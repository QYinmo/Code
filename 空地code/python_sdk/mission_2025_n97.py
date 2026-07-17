import tkinter as tk
from tkinter import messagebox
import os
import threading
import time
from datetime import datetime
import queue
from mission_2025_draft import xy_to_ab, id_to_xy,ab_to_xy,xy_to_num
from usr_serial import Serial_station, Serial_gpio
from mission_route_plan import DroneMissionPlanner

class GroundStationUI:
    def __init__(self, master):
        self.master = master
        master.title("野生动物巡查系统 - 地面站")

        master.attributes('-fullscreen', True)

        self.screen_width_pixels = master.winfo_screenwidth()
        self.screen_height_pixels = master.winfo_screenheight()

        # 调整 grid_size 使地图更小
        self.grid_size = 40  # 从 70 减小
        self.rows = 7
        self.cols = 9
        self.current_selected_cell = None

        self.no_fly_zones = set()
        self.animal_detections = {}
        self.start_coords = (6, 8)
        self.planned_path = []  # 存储规划好的路径

        # 新增/修改用于数据接收的变量
        self.serial_data_queue = queue.Queue()
        self.animal_names = ["象", "虎", "狼", "猴", "孔雀"]
        self.running_serial_thread = True
        self.serial_thread = None

        self.create_widgets()

        master.bind("<Escape>", self.exit_fullscreen)

        self.no_fly_zone_input_stage = "A"
        self.current_no_fly_cell_input = {"A": "", "B": ""}
        self.status_label.config(text="状态: 准备设置禁飞区 - 请输入 A 列数字 (1-9)")

        # 串口初始化
        self.serial_terminal = Serial_station(device="cp2102", baudrate=115200, rx_length=9)
        self.terminal_com = [170, 0, 255] #起飞信号

        # 启动串口监听线程
        self.start_serial_listener()

    def start_serial_listener(self):
        """启动串口监听线程。"""
        self.running_serial_thread = True
        self.serial_thread = threading.Thread(target=self._serial_read_thread, daemon=True)
        self.serial_thread.start()
        self.master.after(100, self.check_serial_queue)

    def _serial_read_thread(self):
        """
        串口读取线程的目标函数。
        """
        local_rx_buffer = [0, 0, 0, 0, 0, 0, 0]
        self.serial_terminal.listen_start(local_rx_buffer)

        while self.running_serial_thread:
            if any(local_rx_buffer[i] > 0 for i in range(2, 7)):
                self.serial_data_queue.put(list(local_rx_buffer))
                for i in range(2, 7):
                    local_rx_buffer[i] = 0
            time.sleep(0.1)

    def create_widgets(self):
        # 顶部区域
        self.top_frame = tk.Frame(self.master, bd=2, relief="groove")
        self.top_frame.pack(side=tk.TOP, fill=tk.X, padx=10, pady=5)

        self.title_label = tk.Label(self.top_frame, text="野生动物巡查系统 地面站", font=("song ti", 18, "bold"))
        self.title_label.pack(side=tk.LEFT, pady=5, padx=10)

        self.exit_button = tk.Button(self.top_frame, text="退出全屏 (ESC)", command=self.exit_fullscreen, font=("song ti", 10))
        self.exit_button.pack(side=tk.RIGHT, pady=5, padx=10)

        # 主内容区域
        self.main_frame = tk.Frame(self.master)
        self.main_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=10, pady=5)

        # 左侧：航线图 Canvas 区域
        self.canvas_frame = tk.Frame(self.main_frame, bd=2, relief="solid")
        self.canvas_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.BOTH, expand=False) # 将 expand 改为 False

        # 调整 canvas_width 和 canvas_height 以使其更小
        canvas_width = self.cols * self.grid_size + 20
        canvas_height = self.rows * self.grid_size + 20

        self.canvas = tk.Canvas(self.canvas_frame, bg="lightgray", width=canvas_width, height=canvas_height)
        self.canvas.pack(padx=10, pady=10) # 移除 expand=True
        self.draw_grid()

        # 右侧：信息显示和按键区
        self.info_panel_frame = tk.Frame(self.main_frame, bd=2, relief="groove")
        self.info_panel_frame.pack(side=tk.RIGHT, padx=5, pady=5, fill=tk.Y)

        self.info_panel_frame.pack_propagate(False)
        self.info_panel_frame.config(width=350)

        # 动物识别与统计显示
        self.detection_label = tk.Label(self.info_panel_frame, text="实时动物检测：", font=("song ti", 14, "bold"))
        self.detection_label.pack(pady=5, padx=5, anchor=tk.W)

        # 添加一个新的Frame来包含Text和Scrollbar
        self.detection_text_frame = tk.Frame(self.info_panel_frame)
        self.detection_text_frame.pack(pady=5, padx=5, expand=True, fill=tk.BOTH)

        self.detection_text = tk.Text(self.detection_text_frame, width=35, height=15, state=tk.DISABLED, wrap=tk.WORD, font=("song ti", 9))
        self.detection_text.pack(side=tk.LEFT, expand=True, fill=tk.Y)
        
        # 创建垂直滚动条
        self.scrollbar = tk.Scrollbar(self.detection_text_frame, command=self.detection_text.yview)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # 将滚动条和Text组件关联
        self.detection_text.config(yscrollcommand=self.scrollbar.set)

        self.total_animals_label = tk.Label(self.info_panel_frame, text="总动物数量：", font=("song ti", 14, "bold"))
        self.total_animals_label.pack(pady=5, padx=5, anchor=tk.W)

        # 数据保存按钮和起飞按钮
        self.action_buttons_frame = tk.Frame(self.info_panel_frame)
        self.action_buttons_frame.pack(pady=10)

        self.save_txt_button = tk.Button(self.action_buttons_frame, text="保存报告 (TXT)", command=self.save_animal_report_to_txt, font=("song ti", 10))
        self.save_txt_button.pack(side=tk.LEFT, padx=8)

        # 新增的"规划航线"按钮
        self.plan_button = tk.Button(self.action_buttons_frame, text="规划航线", command=self.plan_mission_path, font=("song ti", 10), bg="blue", fg="white")
        self.plan_button.pack(side=tk.LEFT, padx=8)

        # 新增的"起飞"按钮
        self.takeoff_button = tk.Button(self.action_buttons_frame, text="起飞", command=self.send_takeoff_signal, font=("song ti", 10), bg="green", fg="white")
        self.takeoff_button.pack(side=tk.LEFT, padx=8)

        # 按键输入设备
        self.key_input_frame = tk.Frame(self.info_panel_frame, bd=2, relief="flat")
        self.key_input_frame.pack(pady=(10, 5)) # 调整 pady，上方 10，下方 5

        key_values = [
            '7', '8', '9',
            '4', '5', '6',
            '1', '2', '3',
            '清空', '0', '确定'
        ]
        row_idx = 1
        col_idx = 0
        for key in key_values:
            button = tk.Button(self.key_input_frame, text=key, width=8, height=2, font=("song ti", 12),
                                command=lambda k=key: self.handle_key_press(k))
            button.grid(row=row_idx, column=col_idx, padx=2, pady=2)
            col_idx += 1
            if col_idx > 2:
                col_idx = 0
                row_idx += 1

        self.status_label = tk.Label(self.info_panel_frame, text="状态: 等待操作...", bd=1, relief="sunken", anchor=tk.W, font=("song ti", 10), height=2)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

    def cell_code_to_coords(self, cell_code):
        try:
            parts = cell_code.split('B')
            col_num = int(parts[0][1:])
            row_num = int(parts[1])

            row_idx = self.rows - row_num
            col_idx = col_num - 1
            return (row_idx, col_idx)
        except (ValueError, IndexError):
            return None

    def coords_to_cell_code(self, coords):
        row_idx, col_idx = coords
        if not (0 <= row_idx < self.rows and 0 <= col_idx < self.cols):
            return None
        col_num = col_idx + 1
        row_num = self.rows - row_idx
        return f"A{col_num}B{row_num}"

    def draw_grid(self):
        self.canvas.delete("all")

        # 绘制方格
        for r in range(self.rows):
            for c in range(self.cols):
                x1 = c * self.grid_size + 10
                y1 = r * self.grid_size + 10
                x2 = x1 + self.grid_size
                y2 = y1 + self.grid_size
                cell_code = f"A{c+1}B{self.rows-r}"

                fill_color = "white"
                if cell_code in self.no_fly_zones:
                    fill_color = "gray"

                self.canvas.create_rectangle(x1, y1, x2, y2, outline="gray", fill=fill_color, tags=cell_code)
                self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=cell_code, font=("song ti", 10, "bold"), tags=cell_code)

        # 绘制起始区域 (红色)
        start_row, start_col = self.start_coords
        x1 = start_col * self.grid_size + 10
        y1 = start_row * self.grid_size + 10
        x2 = x1 + self.grid_size
        y2 = y1 + self.grid_size
        self.canvas.create_oval(x1, y1, x2, y2, fill="red", outline="red", width=2)

        # 绘制规划路径
        if self.planned_path:
            for i in range(len(self.planned_path)-1):
                start_row, start_col = self.planned_path[i]
                end_row, end_col = self.planned_path[i+1]

                start_x = start_col * self.grid_size + 10 + self.grid_size/2
                start_y = start_row * self.grid_size + 10 + self.grid_size/2
                end_x = end_col * self.grid_size + 10 + self.grid_size/2
                end_y = end_row * self.grid_size + 10 + self.grid_size/2

                self.canvas.create_line(start_x, start_y, end_x, end_y,
                                        fill="blue", width=3, arrow=tk.LAST,
                                        arrowshape=(8, 10, 5))

    def handle_key_press(self, key):
        if key.isdigit():
            if self.no_fly_zone_input_stage == "A":
                if 1 <= int(key) <= 9:
                    self.current_no_fly_cell_input["A"] = key
                    self.status_label.config(text=f"状态: 已输入 A{key}。请按 '确定' 进入 B 列输入。")
                else:
                    self.status_label.config(text="错误: A 列请输入 1-9 的数字！")
            elif self.no_fly_zone_input_stage == "B":
                if 1 <= int(key) <= 7:
                    self.current_no_fly_cell_input["B"] = key
                    self.status_label.config(text=f"状态: 已输入 B{key}。请按 '确定' 完成设置。")
                else:
                    self.status_label.config(text="错误: B 列请输入 1-7 的数字！")
        elif key == '清空':
            self.no_fly_zone_input_stage = "A"
            self.current_no_fly_cell_input = {"A": "", "B": ""}
            self.status_label.config(text="状态: 请重新输入 A 列数字 (1-9)。")
        elif key == '确定':
            if self.no_fly_zone_input_stage == "A":
                if self.current_no_fly_cell_input["A"]:
                    self.no_fly_zone_input_stage = "B"
                    self.status_label.config(text=f"状态: 请输入禁飞区 B 列数字 (1-7) for A{self.current_no_fly_cell_input['A']}。")
                else:
                    self.status_label.config(text="错误: 请先输入 A 列数字！")
            elif self.no_fly_zone_input_stage == "B":
                if self.current_no_fly_cell_input["A"] and self.current_no_fly_cell_input["B"]:
                    cell_code = f"A{self.current_no_fly_cell_input['A']}B{self.current_no_fly_cell_input['B']}"

                    if self.cell_code_to_coords(cell_code) == self.start_coords:
                        messagebox.showwarning("错误", "无法将起始点设置为禁飞区！")
                        self.status_label.config(text="状态: 错误：起始点不能设为禁飞区。")
                        self.no_fly_zone_input_stage = "A"
                        self.current_no_fly_cell_input = {"A": "", "B": ""}
                        self.status_label.config(self.status_label.cget("text") + " 请输入下一个禁飞区 A 列数字 (1-9)。")
                        return

                    if cell_code in self.no_fly_zones:
                        self.no_fly_zones.remove(cell_code)
                        self.status_label.config(text=f"状态: 禁飞区 {cell_code} 已移除。")
                    else:
                        self.no_fly_zones.add(cell_code)
                        self.status_label.config(text=f"状态: 禁飞区 {cell_code} 已设置。")

                    self.draw_grid()
                    self.no_fly_zone_input_stage = "A"
                    self.current_no_fly_cell_input = {"A": "", "B": ""}
                    current_text = self.status_label.cget("text")
                    self.status_label.config(text=f"{current_text.split('。')[0]}。 请输入下一个禁飞区 A 列数字 (1-9)。")
                else:
                    self.status_label.config(text="错误: A 列和 B 列都必须输入！")

    def plan_mission_path(self):
        """规划无人机航线路径"""
        if not self.no_fly_zones:
            messagebox.showinfo("提示", "当前未设置任何禁飞区，将规划完整区域的航线。")

        # 将禁飞区从AB格式转换为xy坐标
        forbidden_cells = set()
        for cell_code in self.no_fly_zones:
            parts = cell_code.split('B')
            a_num = int(parts[0][1:])
            b_num = int(parts[1])
            code = ab_to_xy[(a_num,b_num)] # 修正: 使用方括号访问字典
            if code:
                forbidden_cells.add(code)

        # 将离散的禁飞区单元格合并为1x3或3x1的区块
        forbidden_blocks = self._merge_forbidden_cells(forbidden_cells)

        # 创建路径规划器
        planner = DroneMissionPlanner(self.rows, self.cols)

        try:
            # 规划路径
            path = planner.plan_and_visualize_mission(forbidden_blocks)

            if path:
                self.planned_xy_path=path[:]
                self.planned_ab_path=[]
                self.planned_path = []
                for x,y in path:
                    ab=xy_to_ab[(x,y)]
                    a_num, b_num = ab
            # 将 (A_num, B_num) 格式化成 "A{num}B{num}" 字符串
                    ab_string = f"A{a_num}B{b_num}"
                    self.planned_ab_path.append(ab_string)

                for i in range(len(self.planned_ab_path)):
                    self.planned_path.append(self.cell_code_to_coords(self.planned_ab_path[i]))
                self.draw_grid()  # 重绘网格和路径

                # 打印路径信息
                path_info = "规划航线路径:\n"
                for i, (row, col) in enumerate(path):

                    path_info += f"{i+1}. 坐标: ({row}, {col}) \n"

                print(path_info)  # 控制台输出
                self.status_label.config(text="状态: 航线规划完成，路径已显示在地图上")
                messagebox.showinfo("航线规划", "航线规划完成，路径已显示在地图上")
            else:
                self.status_label.config(text="状态: 无法找到有效路径")
                messagebox.showwarning("航线规划", "无法找到有效路径")
        except Exception as e:
            self.status_label.config(text=f"状态: 航线规划错误: {str(e)}")
            messagebox.showerror("航线规划错误", f"发生错误: {str(e)}")

    def _merge_forbidden_cells(self, forbidden_cells):
        """
        将离散的禁飞区单元格合并为1x3或3x1的区块
        返回符合DroneMissionPlanner要求的区块列表
        """
        forbidden_blocks = []
        processed_cells = set()

        # 首先尝试合并水平方向的1x3区块
        for (r, c) in forbidden_cells:
            if (r, c) in processed_cells:
                continue

            # 检查右侧是否有连续3个单元格
            if (r, c+1) in forbidden_cells and (r, c+2) in forbidden_cells:
                block = [(r, c), (r, c+1), (r, c+2)]
                forbidden_blocks.append(block)
                processed_cells.update(block)
                continue

            # 检查左侧是否有连续3个单元格
            if (r, c-1) in forbidden_cells and (r, c-2) in forbidden_cells:
                block = [(r, c-2), (r, c-1), (r, c)]
                forbidden_blocks.append(block)
                processed_cells.update(block)
                continue

        # 然后尝试合并垂直方向的3x1区块
        for (r, c) in forbidden_cells:
            if (r, c) in processed_cells:
                continue

            # 检查下方是否有连续3个单元格
            if (r+1, c) in forbidden_cells and (r+2, c) in forbidden_cells:
                block = [(r, c), (r+1, c), (r+2, c)]
                forbidden_blocks.append(block)
                processed_cells.update(block)
                continue

            # 检查上方是否有连续3个单元格
            if (r-1, c) in forbidden_cells and (r-2, c) in forbidden_cells:
                block = [(r-2, c), (r-1, c), (r, c)]
                forbidden_blocks.append(block)
                processed_cells.update(block)
                continue

        # 最后处理剩余的单个或两个单元格（作为1x1或2x1区块处理）
        remaining_cells = forbidden_cells - processed_cells
        for cell in remaining_cells:
            forbidden_blocks.append([cell])  # 作为1x1区块处理

        print("禁飞区块转换结果:")
        for i, block in enumerate(forbidden_blocks):
            block_codes = [self.coords_to_cell_code(c) for c in block]
            print(f"区块{i+1}: {block} (AB格式: {block_codes})")

        return forbidden_blocks
    def update_animal_detection(self, cell_code, animal_type, count):
        if cell_code not in self.animal_detections:
            self.animal_detections[cell_code] = {}

        if animal_type not in self.animal_detections[cell_code]:
            self.animal_detections[cell_code][animal_type] = 0

        self.animal_detections[cell_code][animal_type] = count

    def display_detections(self):
        self.detection_text.config(state=tk.NORMAL)
        self.detection_text.delete(1.0, tk.END)

        # 设置行间距的标签配置
        self.detection_text.tag_configure("spacing", spacing1=5, spacing3=5)  # 增加上下间距

        total_animals_count = 0
        animal_summary = {}

        sorted_cells = sorted(self.animal_detections.keys())

        for cell_code in sorted_cells:
            animals = self.animal_detections[cell_code]
            # 使用标签应用行间距
            self.detection_text.insert(tk.END, f"方格 {cell_code}:\n\n", "spacing")
            for animal in sorted(animals.keys()):
                count = animals[animal]
                self.detection_text.insert(tk.END, f"- {animal}: {count}只\n\n", "spacing")
                total_animals_count += count
                if animal not in animal_summary:
                    animal_summary[animal] = 0
                animal_summary[animal] += count
            self.detection_text.insert(tk.END, "\n", "spacing")

        self.detection_text.config(state=tk.DISABLED)

        total_summary_str = "总动物数量：\n\n"  # 增加换行符
        sorted_animals = sorted(animal_summary.keys())
        for animal in sorted_animals:
            count = animal_summary[animal]
            total_summary_str += f" - {animal}: {count}只\n\n"  # 增加换行符
        total_summary_str += f"\n总计: {total_animals_count}只"
        self.total_animals_label.config(text=total_summary_str)

        self.status_label.config(text="状态: 动物检测数据已更新")

    def save_animal_report_to_txt(self):
            try:
                file_name = "野生动物巡查报告.txt"
                # 动态获取桌面路径
                desktop_path = os.path.join(os.path.expanduser('~'), 'Desktop')
                # 确保桌面目录存在
                if not os.path.exists(desktop_path):
                    # 如果是macOS，桌面路径可能不存在，但通常在用户主目录下，直接保存到主目录
                    desktop_path = os.path.expanduser('~')
                    
                file_path = os.path.join(desktop_path, file_name)

                report_content = []
                report_content.append(f"======== 野生动物巡查报告 ========\n")
                report_content.append(f"生成时间: {datetime.now().strftime('%Y年%m月%d日 %H:%M:%S')}\n\n")

                report_content.append("--- 禁飞区设置 ---\n")
                if self.no_fly_zones:
                    sorted_no_fly = sorted(list(self.no_fly_zones))
                    report_content.append(f"已设置禁飞区 ({len(sorted_no_fly)} 个): {', '.join(sorted_no_fly)}\n")
                else:
                    report_content.append("当前无禁飞区设置。\n")
                report_content.append("\n")

                report_content.append("--- 动物检测数据 ---\n")
                if self.animal_detections:
                    total_all_animals = 0
                    animal_summary_overall = {}

                    sorted_cells = sorted(self.animal_detections.keys())
                    for cell_code in sorted_cells:
                        animals = self.animal_detections[cell_code]
                        report_content.append(f"方格 {cell_code}:\n")
                        for animal in sorted(animals.keys()):
                            count = animals[animal]
                            report_content.append(f"  - {animal}: {count}只\n")
                            total_all_animals += count
                            if animal not in animal_summary_overall:
                                animal_summary_overall[animal] = 0
                            animal_summary_overall[animal] += count
                        report_content.append("\n")

                    report_content.append("--- 动物总计 ---\n")
                    for animal in sorted(animal_summary_overall.keys()):
                        count = animal_summary_overall[animal]
                        report_content.append(f"  - {animal}: {count}只\n")
                    report_content.append(f"总计检测到动物: {total_all_animals}只\n")
                else:
                    report_content.append("当前无动物检测数据。\n")

                with open(file_path, 'w', encoding='utf-8') as f:
                    f.writelines(report_content)

                messagebox.showinfo("保存成功", f"动物巡查报告已保存到:\n{file_path}\n(此文本文件已被覆盖，用于事后查看)")
                self.status_label.config(text="状态: 动物巡查报告已保存为文本文件。")
            except Exception as e:
                messagebox.showerror("保存失败", f"保存文本报告时发生错误: {e}")
                self.status_label.config(text="状态: 文本报告保存失败。")

    def check_serial_queue(self):
        data_processed_in_this_cycle = False
        while not self.serial_data_queue.empty():
            try:
                rx_data = self.serial_data_queue.get_nowait()
                col_num = rx_data[0]
                col_num= int(str(col_num).split('.')[0])
                row_num = rx_data[1]
                row_num= int(str(row_num).split('.')[0])
                cell_code = f"A{col_num}B{row_num}"

                for i, animal_type in enumerate(self.animal_names):
                    count = rx_data[i + 2]
                    count= int(str(count).split('.')[0])
                    if count > 0:
                        self.update_animal_detection(cell_code, animal_type, count)
                        data_processed_in_this_cycle = True

            except queue.Empty:
                break
            except Exception as e:
                print(f"处理队列数据时发生错误: {e}")
                self.status_label.config(text=f"状态: 数据处理错误: {e}")

        if data_processed_in_this_cycle:
            self.display_detections()

        if self.running_serial_thread:
            self.master.after(100, self.check_serial_queue)

    def send_takeoff_signal(self):
        self.terminal_com[1] = 1
        waypoint=[]
        for x,y in self.planned_xy_path:
            waypoint.append(xy_to_num[(x,y)])  # 示例航点数据，实际应根据规划路径生成
        message_to_send = [self.terminal_com[0], 1] + waypoint + [self.terminal_com[-1]]
        print(f"{message_to_send}")
        try:
            self.serial_terminal.send_start(message_to_send)
            self.status_label.config(text=f"状态: 已发送起飞信号: {self.terminal_com}")
            messagebox.showinfo("起飞信号", "已成功发送起飞信号！")
        except Exception as e:
            messagebox.showerror("串口发送错误", f"发送起飞信号时发生错误: {e}")
            self.status_label.config(text=f"状态: 发送起飞信号失败: {e}")

    def exit_fullscreen(self, event=None):
        self.running_serial_thread = False
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
        if hasattr(self.serial_terminal, 'stop_transmit'):
            self.serial_terminal.stop_transmit()
        self.master.attributes('-fullscreen', False)
        self.master.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = GroundStationUI(root)
    root.mainloop()