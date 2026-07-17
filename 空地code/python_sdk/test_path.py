import matplotlib.pyplot as plt
import matplotlib.patches as patches
from collections import deque

# --- 1. 网格表示和禁飞区整合 ---
def initialize_grid():
    """
    初始化9x7的网格，并定义方格的边界。
    网格坐标系统：
    - 行 (y轴): B1-B7 对应 0-6
    - 列 (x轴): A1-A9 对应 0-8
    巡查区域为450cm × 350cm，分成63个50cm × 50cm的方格。
    """
    rows = 7 # B1-B7
    cols = 9 # A1-A9
    grid = [["OPEN" for _ in range(cols)] for _ in range(rows)]
    return grid, rows, cols

def convert_code_to_coords(code):
    """
    将方格代码（如“A3B5”）转换为0索引的网格坐标（行，列）。
    B1对应行0，A1对应列0。
    """
    if len(code) != 4 or not (code[0].isalpha() and code[1].isdigit() and code[2].isalpha() and code[3].isdigit()):
        raise ValueError("Invalid square code format. Expected 'AnBm'.")
    
    col_char = code[0].upper() # A-I
    row_char = code[2].upper() # B
    row_num = int(code[3])     # 1-7

    col = ord(col_char) - ord('A')
    row = row_num - 1 # B1 -> row 0

    if not (0 <= col < 9 and 0 <= row < 7):
        raise ValueError(f"Square code {code} is out of bounds.")
    
    return row, col

def convert_coords_to_code(r, c):
    """
    将0索引的网格坐标（行，列）转换为方格代码（如“A3B5”）。
    """
    if not (0 <= r < 7 and 0 <= c < 9):
        raise ValueError(f"Coordinates ({r}, {c}) are out of bounds.")
    return f"{chr(ord('A')+c)}{chr(ord('B')+r+1)}"

def set_no_fly_zones(grid, no_fly_codes):
    """
    在网格中设置禁飞区。禁飞区由数个连续方格组成。
    """
    for code in no_fly_codes:
        r, c = convert_code_to_coords(code)
        grid[r][c] = "BLOCKED"
    print(f"禁飞区已设置: {no_fly_codes}")
    return grid

# --- BFS 辅助函数：寻找最短四方向路径 ---
def bfs_shortest_path(start, end, grid, visited_in_current_sweep):
    """
    使用BFS在网格中寻找从start到end的最短路径，只沿四个方向移动，
    避开禁飞区和在当前sweep中已访问的方格。
    start, end: (row, col) 元组
    grid: 2D 列表，表示网格状态
    visited_in_current_sweep: 2D 布尔列表，标记在本次规划中已经访问过的方格
    """
    rows, cols = len(grid), len(grid[0])
    queue = deque([(start, [start])]) # (current_pos, path_to_current_pos)
    
    # visited 集合用于BFS内部，防止重复访问导致死循环或非最短路径
    bfs_visited = set()
    bfs_visited.add(start)

    # 四个方向：上，下，左，右 (dr, dc)
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while queue:
        (r, c), path = queue.popleft()

        if (r, c) == end:
            return path

        for dr, dc in directions:
            nr, nc = r + dr, c + dc

            # 检查新位置是否在网格内
            if 0 <= nr < rows and 0 <= nc < cols:
                # 检查新位置是否未被访问过（BFS内部）且不是禁飞区
                # 并且不应该是在本次整个路径规划中已经访问过的方格，除非是终点
                if (nr, nc) not in bfs_visited and grid[nr][nc] != "BLOCKED":
                    # 避免重新访问已经规划到主路径的方格，除非它是目标方格
                    if not visited_in_current_sweep[nr][nc] or (nr, nc) == end:
                        bfs_visited.add((nr, nc))
                        new_path = path + [(nr, nc)]
                        queue.append(((nr, nc), new_path))
    return None # 没有找到路径

# --- 2. 路径规划策略（修改后的蛇形模式）---
def plan_cardinal_path_with_obstacles(grid, rows, cols, start_pos_code="A9B1"):
    """
    规划蛇形遍历路径，从指定起点开始，绕过禁飞区，只沿四个方向移动。
    """
    full_path = []
    # visited_overall 用于跟踪整个巡查过程中已经访问过的方格
    visited_overall = [[False for _ in range(cols)] for _ in range(rows)]
    
    start_row, start_col = convert_code_to_coords(start_pos_code)
    current_pos = (start_row, start_col)

    # 如果起飞点是禁飞区，则无法规划
    if grid[start_row][start_col] == "BLOCKED":
        print(f"警告：起始点 {start_pos_code} 位于禁飞区。无法规划路径。")
        return []

    # 将起飞点加入路径并标记为已访问
    full_path.append(current_pos)
    visited_overall[start_row][start_col] = True

    # 生成按蛇形顺序的理想巡查点列表
    # 从B1行开始，向右，然后Z字形
    desired_sweep_order = []
    direction_col = 1 # 1 for A1->A9, -1 for A9->A1
    for r_idx in range(rows):
        if direction_col == 1:
            for c_idx in range(cols):
                desired_sweep_order.append((r_idx, c_idx))
        else:
            for c_idx in range(cols - 1, -1, -1):
                desired_sweep_order.append((r_idx, c_idx))
        direction_col *= -1 # 改变下一行的方向

    # 过滤掉禁飞区，并且确保不要重复添加起飞点（如果它在巡查序列中）
    filtered_sweep_order = []
    for r, c in desired_sweep_order:
        if grid[r][c] != "BLOCKED":
            if (r, c) != current_pos: # 避免重复添加起点
                filtered_sweep_order.append((r, c))
    
    # 逐个访问未访问的方格
    for target_r, target_c in filtered_sweep_order:
        if not visited_overall[target_r][target_c]:
            path_segment = bfs_shortest_path(current_pos, (target_r, target_c), grid, visited_overall)
            
            if path_segment is None:
                # 这表示有不可达的方格，或者路径被完全阻断
                print(f"警告：无法从 {convert_coords_to_code(current_pos[0], current_pos[1])} 到达 {convert_coords_to_code(target_r, target_c)}。路径可能不完整。")
                continue # 跳过此目标，尝试下一个

            # 将路径段添加到完整路径中，并标记为已访问
            # 跳过路径段的第一个点，因为它就是当前位置，避免重复
            for r, c in path_segment[1:]: 
                full_path.append((r, c))
                visited_overall[r][c] = True
            current_pos = (target_r, target_c) # 更新当前位置

    return full_path

# --- 3. 可视化部分 ---
def visualize_path(grid, path, start_pos_code, no_fly_codes):
    """
    可视化网格、规划路径、禁飞区和起飞区域。
    在显示屏按9×7方格画出巡查航线。
    """
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # 定义方格大小和颜色
    cell_size = 1.0 # 每个方格的相对大小
    
    # 绘制网格线和方格编码
    rows = len(grid) 
    cols = len(grid[0]) if rows > 0 else 0 
    
    for r in range(rows):
        for c in range(cols):
            # 绘制方格边框
            rect = patches.Rectangle((c * cell_size, r * cell_size), cell_size, cell_size,
                                     linewidth=0.5, edgecolor='gray', facecolor='none')
            ax.add_patch(rect)
            
            # 添加方格编码 (e.g., A1B1)
            square_code = f"{chr(ord('A')+c)}{chr(ord('B')+r+1)}"
            ax.text(c * cell_size + 0.5 * cell_size, r * cell_size + 0.5 * cell_size,
                    square_code, ha='center', va='center', fontsize=8, color='darkgray')

            # 标记禁飞区 (灰色矩形)
            if grid[r][c] == "BLOCKED":
                rect_blocked = patches.Rectangle((c * cell_size, r * cell_size), cell_size, cell_size,
                                                 linewidth=0, edgecolor='none', facecolor='lightgray', alpha=0.7)
                ax.add_patch(rect_blocked)
                ax.text(c * cell_size + 0.5 * cell_size, r * cell_size + 0.5 * cell_size,
                        "禁飞区", ha='center', va='center', fontsize=7, color='red')

    # 绘制起飞区域 (红色起飞区域)
    start_row, start_col = convert_code_to_coords(start_pos_code)
    start_circle = patches.Circle((start_col * cell_size + 0.5 * cell_size, start_row * cell_size + 0.5 * cell_size),
                                  radius=0.4 * cell_size, color='red', alpha=0.6)
    ax.add_patch(start_circle)
    ax.text(start_col * cell_size + 0.5 * cell_size, start_row * cell_size + 0.5 * cell_size,
            "起飞", ha='center', va='center', color='white', fontsize=9, fontweight='bold')

    # 绘制规划路径
    if path:
        # 路径点的x, y坐标
        path_x = [p[1] * cell_size + 0.5 * cell_size for p in path]
        path_y = [p[0] * cell_size + 0.5 * cell_size for p in path]
        
        # 将起飞点作为路径的起点，并用虚线连接到实际规划路径的第一个点
        if path and (path[0][0] != start_row or path[0][1] != start_col):
            # 添加从起飞点到路径第一个点的连接线
            ax.plot([start_col * cell_size + 0.5 * cell_size, path_x[0]],
                    [start_row * cell_size + 0.5 * cell_size, path_y[0]],
                    'k--', linewidth=1, alpha=0.7, label='起飞连接线')

        ax.plot(path_x, path_y, 'b-o', linewidth=2, markersize=5, markerfacecolor='blue', label='规划航线')
        
        # 标记路径方向
        for i in range(len(path_x) - 1):
            dx = path_x[i+1] - path_x[i]
            dy = path_y[i+1] - path_y[i]
            # 只有当有实际移动时才绘制箭头
            if abs(dx) > 0.01 or abs(dy) > 0.01: # 避免绘制起点自身的箭头
                ax.arrow(path_x[i], path_y[i], dx * 0.8, dy * 0.8,
                         head_width=0.1, head_length=0.1, fc='blue', ec='blue')

    ax.set_xlim(0, cols * cell_size)
    ax.set_ylim(0, rows * cell_size)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect('equal', adjustable='box')
    ax.set_title("野生动物巡查系统 - 规划航线可视化 (H题)", fontsize=14)
    ax.legend()
    ax.grid(False) # 关闭默认的网格线，我们自己绘制了
    plt.show()

# --- 测试用例和用户交互 ---
if __name__ == "__main__":
    while True:
        print("\n--- 无人机路径规划模拟 ---")
        print("请选择操作：")
        print("1. 运行预设测试场景")
        print("2. 自定义禁飞区")
        print("3. 退出")
        
        choice = input("请输入你的选择 (1/2/3): ")

        if choice == '1':
            # 场景1: 无禁飞区
            print("\n--- 测试场景 1: 无禁飞区 ---")
            initial_grid_s1, rows_s1, cols_s1 = initialize_grid()
            no_fly_zones_scenario1 = []
            grid_s1 = set_no_fly_zones(initial_grid_s1, no_fly_zones_scenario1)
            path_s1 = plan_cardinal_path_with_obstacles(grid_s1, rows_s1, cols_s1, start_pos_code="A9B1")
            print(f"规划路径方格数量: {len(path_s1)}")
            visualize_path(grid_s1, path_s1, "A9B1", no_fly_zones_scenario1)

            # 场景2: 禁飞区在中间 (图1所示的禁飞区示例)
            print("\n--- 测试场景 2: 禁飞区在中间 (类似图1) ---")
            no_fly_zones_scenario2 = ["A3B3", "A4B3", "A5B3"] # 对应图1中的灰色矩形
            initial_grid_s2, rows_s2, cols_s2 = initialize_grid() 
            grid_s2 = set_no_fly_zones(initial_grid_s2, no_fly_zones_scenario2)
            path_s2 = plan_cardinal_path_with_obstacles(grid_s2, rows_s2, cols_s2, start_pos_code="A9B1")
            print(f"规划路径方格数量: {len(path_s2)}")
            visualize_path(grid_s2, path_s2, "A9B1", no_fly_zones_scenario2)

            # 场景3: 禁飞区在边缘，且起始点在禁飞区外
            print("\n--- 测试场景 3: 禁飞区在边缘 ---")
            no_fly_zones_scenario3 = ["A1B7", "A2B7", "A1B6"] 
            initial_grid_s3, rows_s3, cols_s3 = initialize_grid()
            grid_s3 = set_no_fly_zones(initial_grid_s3, no_fly_zones_scenario3)
            path_s3 = plan_cardinal_path_with_obstacles(grid_s3, rows_s3, cols_s3, start_pos_code="A9B1")
            print(f"规划路径方格数量: {len(path_s3)}")
            visualize_path(grid_s3, path_s3, "A9B1", no_fly_zones_scenario3)

            # 场景4: 更多禁飞区
            print("\n--- 测试场景 4: 更多禁飞区 ---")
            no_fly_zones_scenario4 = ["A2B2", "A3B2", "A4B2", "A5B4", "A6B4", "A7B4", "A8B6", "A9B6"]
            initial_grid_s4, rows_s4, cols_s4 = initialize_grid()
            grid_s4 = set_no_fly_zones(initial_grid_s4, no_fly_zones_scenario4)
            path_s4 = plan_cardinal_path_with_obstacles(grid_s4, rows_s4, cols_s4, start_pos_code="A9B1")
            print(f"规划路径方格数量: {len(path_s4)}")
            visualize_path(grid_s4, path_s4, "A9B1", no_fly_zones_scenario4)

        elif choice == '2':
            print("\n--- 自定义禁飞区 ---")
            print("请输入禁飞区的方格代码，多个代码之间用逗号分隔（例如：A3B3,A4B3,A5B3）。")
            print("输入 'done' 结束。")
            
            custom_no_fly_input = input("请输入禁飞区代码：").strip()
            custom_no_fly_zones = []
            if custom_no_fly_input.lower() != 'done' and custom_no_fly_input:
                custom_no_fly_zones = [code.strip().upper() for code in custom_no_fly_input.split(',')]
            
            initial_grid_custom, rows_custom, cols_custom = initialize_grid()
            grid_custom = set_no_fly_zones(initial_grid_custom, custom_no_fly_zones)
            
            # 询问自定义起飞点
            custom_start_pos = input("请输入起飞区域方格代码 (例如：A9B1，留空则默认): ").strip().upper()
            if not custom_start_pos:
                custom_start_pos = "A9B1"

            path_custom = plan_cardinal_path_with_obstacles(grid_custom, rows_custom, cols_custom, start_pos_code=custom_start_pos)
            print(f"规划路径方格数量: {len(path_custom)}")
            visualize_path(grid_custom, path_custom, custom_start_pos, custom_no_fly_zones)

        elif choice == '3':
            print("退出程序。")
            break
        else:
            print("无效选择，请重新输入。")