import tkinter as tk
from tkinter import messagebox


class GridPathSolverGUI:
    def __init__(self, master, rows=7, cols=9):
        self.master = master
        master.title("7x9 格子路径绘制器")

        self.rows = rows
        self.cols = cols
        self.start_pos = (0, 0)  # (x, y) 坐标

        self.path_points = [self.start_pos]  # 存储路径上的所有点 (包括重复)
        self.visited_unique_points = {self.start_pos}  # 存储所有访问过的唯一格子
        self.current_pos = self.start_pos
        self.total_steps = 0

        self.cell_size = 60  # 每个格子的像素大小
        self.canvas_width = self.cols * self.cell_size
        self.canvas_height = self.rows * self.cell_size

        self.canvas = tk.Canvas(master, width=self.canvas_width,
                                height=self.canvas_height, bg="white", borderwidth=2, relief="groove")
        self.canvas.pack(pady=10)
        self.canvas.bind("<Button-1>", self.on_canvas_click)  # 绑定鼠标左键点击事件

        self.draw_grid()

        # --- 修改从这里开始 ---
        # 信息显示标签，这些需要在使用前创建
        self.info_frame = tk.Frame(master)
        self.info_frame.pack(pady=5)

        self.steps_label = tk.Label(
            self.info_frame, text=f"总步数: {self.total_steps}")
        self.steps_label.pack(side=tk.LEFT, padx=10)

        self.visited_label = tk.Label(
            self.info_frame, text=f"已访问唯一格子: {len(self.visited_unique_points)} / {self.rows * self.cols}")
        self.visited_label.pack(side=tk.LEFT, padx=10)
        # --- 修改到这里结束 ---

        self.update_cells()  # 现在调用 update_cells() 是安全的，因为标签已经存在

        # 按钮（这些可以保持在信息标签之后）
        self.button_frame = tk.Frame(master)
        self.button_frame.pack(pady=10)

        self.reset_button = tk.Button(
            self.button_frame, text="重置路径", command=self.reset_path)
        self.reset_button.pack(side=tk.LEFT, padx=10)

        self.finish_button = tk.Button(
            self.button_frame, text="完成路径并分析", command=self.analyze_path)
        self.finish_button.pack(side=tk.LEFT, padx=10)

        self.info_label = tk.Label(
            master, text="点击格子来绘制路径。\n蓝色: 起点 | 绿色: 当前 | 黄色: 路径 | 红色: 重复访问")
        self.info_label.pack(pady=5)

    def draw_grid(self):
        """绘制网格线和坐标标签。"""
        for i in range(self.rows):
            for j in range(self.cols):
                x1, y1 = j * self.cell_size, i * self.cell_size
                x2, y2 = (j + 1) * self.cell_size, (i + 1) * self.cell_size
                self.canvas.create_rectangle(
                    x1, y1, x2, y2, outline="gray", tags=f"cell_{j}_{i}")
                # 在格子中心显示坐标
                self.canvas.create_text(x1 + self.cell_size // 2, y1 + self.cell_size // 2,
                                        text=f"({j},{i})", font=("Arial", 8), tags=f"coord_{j}_{i}")

    def update_cells(self):
        """根据路径更新格子的颜色。"""
        for r in range(self.rows):
            for c in range(self.cols):
                tag = f"cell_{c}_{r}"  # 注意这里也是(x,y)
                current_cell_color = "white"  # 默认白色

                # 检查是否是起点
                if (c, r) == self.start_pos:
                    current_cell_color = "blue"

                # 检查是否是路径点或重复点
                path_count = self.path_points.count((c, r))
                if path_count > 0:
                    if (c, r) != self.start_pos and path_count > 1:
                        current_cell_color = "red"  # 重复访问的非起点格子
                    elif (c, r) != self.start_pos:  # 确保起点不被黄色覆盖
                        current_cell_color = "yellow"  # 路径上的点

                # 检查是否是当前位置 (最后绘制以覆盖其他颜色)
                if (c, r) == self.current_pos:
                    current_cell_color = "green"

                self.canvas.itemconfig(tag, fill=current_cell_color)

        self.steps_label.config(text=f"总步数: {self.total_steps}")
        self.visited_label.config(
            text=f"已访问唯一格子: {len(self.visited_unique_points)} / {self.rows * self.cols}")

    def on_canvas_click(self, event):
        """处理画布点击事件。"""
        # 计算点击位置对应的格子坐标
        clicked_x = event.x // self.cell_size
        clicked_y = event.y // self.cell_size
        clicked_pos = (clicked_x, clicked_y)

        # 检查是否点击了有效且相邻的格子
        if 0 <= clicked_x < self.cols and 0 <= clicked_y < self.rows:
            dist = abs(self.current_pos[0] - clicked_x) + \
                abs(self.current_pos[1] - clicked_y)
            if dist == 1:  # 曼哈顿距离为1，表示相邻
                self.path_points.append(clicked_pos)
                self.visited_unique_points.add(clicked_pos)
                self.current_pos = clicked_pos
                self.total_steps += 1
                self.update_cells()
            else:
                messagebox.showwarning(
                    "无效移动", f"你只能移动到相邻的格子。\n当前位置: {self.current_pos}\n点击位置: {clicked_pos}")
        else:
            messagebox.showwarning("无效点击", "请点击网格内部的格子。")

    def reset_path(self):
        """重置路径。"""
        self.path_points = [self.start_pos]
        self.visited_unique_points = {self.start_pos}
        self.current_pos = self.start_pos
        self.total_steps = 0
        self.update_cells()
        messagebox.showinfo("路径已重置", "路径已清空，请重新开始绘制。")

    def analyze_path(self):
        """分析路径是否有效并显示结果。"""
        all_visited = (len(self.visited_unique_points)
                       == self.rows * self.cols)
        returned_to_start = (self.current_pos == self.start_pos)

        if not all_visited:
            messagebox.showwarning("路径未完成", "你还没有访问所有格子！请继续绘制。")
            return
        if not returned_to_start:
            messagebox.showwarning("路径未闭合", "你还没有回到起点 (0,0)！请继续绘制。")
            return

        # 计算重复访问的非起点格子
        all_points_in_path_count = {}
        for p in self.path_points:
            all_points_in_path_count[p] = all_points_in_path_count.get(
                p, 0) + 1

        repeated_non_start_points = []
        for p, count in all_points_in_path_count.items():
            if p != self.start_pos and count > 1:
                repeated_non_start_points.append(p)

        result_message = (
            f"恭喜！你成功完成了路径。\n"
            f"总步数: {self.total_steps}\n"
            f"重复访问的非起点格子数量: {len(repeated_non_start_points)}\n"
        )
        if repeated_non_start_points:
            # 使用 set 去重并排序，方便显示
            result_message += f"具体重复格子: {sorted(list(set(repeated_non_start_points)))}"
        else:
            result_message += "没有重复访问的非起点格子。"

        messagebox.showinfo("路径分析结果", result_message)


if __name__ == "__main__":
    root = tk.Tk()
    app = GridPathSolverGUI(root)
    root.mainloop()
