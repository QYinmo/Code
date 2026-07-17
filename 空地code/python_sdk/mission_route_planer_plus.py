import numpy as np
import random
import collections
import matplotlib.pyplot as plt
import time
import sys
import itertools

# --- 辅助函数 ---


def get_all_forbidden_layouts(rows, cols, num_forbidden_rects=2, rect_shapes=((1, 3), (3, 1)), start_node=(0, 0)):
    """
    生成所有可能的 num_forbidden_rects 个不重叠的禁止区域组合。
    每个禁止区域可以是 rect_shapes 中定义的任何形状（例如 1x3 或 3x1）。
    """
    forbidden_rects_all_single_positions = []

    for rect_rows, rect_cols in rect_shapes:
        # 水平摆放 (rect_rows x rect_cols)
        for r in range(rows - rect_rows + 1):
            for c_start in range(cols - rect_cols + 1):
                current_rect_cells = set()
                for i in range(rect_rows):
                    for j in range(rect_cols):
                        cell = (r + i, c_start + j)
                        current_rect_cells.add(cell)
                forbidden_rects_all_single_positions.append(
                    frozenset(current_rect_cells))

        # 垂直摆放 (rect_cols x rect_rows) - 仅当形状不同时才需要额外考虑
        # 例如，如果是 (1,3)，则不需要生成 (3,1) 的“垂直摆放”因为它会通过 (3,1) 形状自身被生成
        # 但是对于 (1,3) 和 (3,1) 两种形状，我们需要确保各自的生成逻辑是独立的。
        # 上面的循环已经覆盖了 (1,3) 作为 rect_rows x rect_cols 的情况
        # 接下来，我们如果rect_rows != rect_cols 考虑 (3,1)作为 rect_rows x rect_cols
        # 对于 (1,3) 和 (3,1) 两种情况，它们会在 for rect_rows, rect_cols in rect_shapes 循环中独立处理。
        # 比如第一次循环 rect_shape=(1,3)，第二次循环 rect_shape=(3,1)
        pass  # 此处无需额外代码，因为循环会处理每种形状

    # 去除重复的单个禁止区域（例如，一个 3x1 的矩形如果与 1x3 的矩形占据同一组单元格，尽管可能性很小，但稳妥起见去重）
    forbidden_rects_all_single_positions = list(
        set(forbidden_rects_all_single_positions))

    # 现在从所有可能的单个位置中选择 num_forbidden_rects 个不重叠的组合
    all_forbidden_layouts_combinations = []

    for combo in itertools.combinations(forbidden_rects_all_single_positions, num_forbidden_rects):
        is_overlapping = False
        forbidden_set_for_combo = set()
        for rect in combo:
            # 检查当前矩形是否与起点重叠
            if start_node in rect:
                is_overlapping = True
                break
            # 检查当前矩形是否与之前已添加的矩形重叠
            if not forbidden_set_for_combo.isdisjoint(rect):
                is_overlapping = True
                break
            forbidden_set_for_combo.update(rect)

        if not is_overlapping:
            all_forbidden_layouts_combinations.append(
                frozenset(forbidden_set_for_combo))  # 使用frozenset作为整体布局

    return list(set(all_forbidden_layouts_combinations))  # 最终去重


# BFS 缓存机制
BFS_CACHE = {}


def bfs_shortest_path_with_cache(start_coords, target_coords, grid_size, forbidden_nodes):
    """
    使用BFS找到从start_coords到target_coords的最短路径列表。
    使用全局缓存避免重复计算。
    """
    forbidden_key = frozenset(forbidden_nodes)
    cache_key = (start_coords, target_coords, forbidden_key)

    if cache_key in BFS_CACHE:
        return BFS_CACHE[cache_key]

    reverse_cache_key = (target_coords, start_coords, forbidden_key)
    if reverse_cache_key in BFS_CACHE:
        path = BFS_CACHE[reverse_cache_key]
        if path:
            return path[::-1]
        else:
            return None

    rows, cols = grid_size
    q = collections.deque([(start_coords, [start_coords])])
    visited = {start_coords}

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # 固定方向顺序

    while q:
        (r, c), path = q.popleft()

        if (r, c) == target_coords:
            BFS_CACHE[cache_key] = path
            return path

        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            next_node_coords = (nr, nc)

            if 0 <= nr < rows and 0 <= nc < cols and \
               next_node_coords not in forbidden_nodes and \
               next_node_coords not in visited:

                visited.add(next_node_coords)
                q.append((next_node_coords, path + [next_node_coords]))

    BFS_CACHE[cache_key] = None
    return None


def generate_boustrophedon_path(rows, cols, start_node=(0, 0)):
    """
    生成一个标准蛇形（Boustrophedon）遍历路径，从(0,0)开始，遍历所有格子，并返回(0,0)。
    """
    path = []
    for r in range(rows):
        if r % 2 == 0:  # 偶数行：从左到右
            for c in range(cols):
                path.append((r, c))
        else:  # 奇数行：从右到左
            for c in range(cols - 1, -1, -1):
                path.append((r, c))

    if path[-1] != start_node:
        path.append(start_node)

    return path

# --- 主求解函数 (基于预设路径和修复) ---


def solve_grid_path_with_refinement(grid_size=(6, 9), forbidden_nodes=frozenset(), start_node=(0, 0)):
    global BFS_CACHE

    rows, cols = grid_size
    BFS_CACHE = {}  # 针对每个新布局清空BFS缓存

    base_path_full = generate_boustrophedon_path(rows, cols, start_node)

    all_reachable_points_set = set()
    for r in range(rows):
        for c in range(cols):
            if (r, c) not in forbidden_nodes:
                all_reachable_points_set.add((r, c))

    if not all_reachable_points_set or (start_node not in all_reachable_points_set):
        return [], float('inf'), False

    final_path = []
    visited_in_final_path = {start_node}
    current_pos = start_node

    # 修正：确保使用 start_node
    final_path.append(start_node)

    unvisited_target_points = all_reachable_points_set.copy()
    unvisited_target_points.discard(start_node)

    remaining_targets_ordered = []
    for p in base_path_full:
        if p not in forbidden_nodes and p != start_node:
            remaining_targets_ordered.append(p)

    target_idx = 0

    while unvisited_target_points:
        next_actual_target = None

        # 1. 尝试从剩余的 base_path_full 目标中寻找下一个可达的未访问点
        current_potential_targets = []
        temp_target_idx = target_idx
        while temp_target_idx < len(remaining_targets_ordered):
            p = remaining_targets_ordered[temp_target_idx]
            if p not in visited_in_final_path:
                current_potential_targets.append(p)
            temp_target_idx += 1

        # 2. 如果基准路径中还有未访问的点，优先考虑它们
        best_candidate_from_base = None
        min_dist_from_base = float('inf')

        for p in current_potential_targets:
            dist_path = bfs_shortest_path_with_cache(
                current_pos, p, grid_size, forbidden_nodes)
            if dist_path is not None:
                dist = len(dist_path) - 1
                if dist < min_dist_from_base:
                    min_dist_from_base = dist
                    best_candidate_from_base = p
                elif dist == min_dist_from_base:
                    if best_candidate_from_base is None or p < best_candidate_from_base:  # 字典序决胜规则
                        best_candidate_from_base = p

        if best_candidate_from_base is not None:
            next_actual_target = best_candidate_from_base
            target_idx = remaining_targets_ordered.index(
                best_candidate_from_base) + 1
        else:
            # 3. 如果基准路径中没有可达的未访问点，则寻找任意最近的未访问点
            closest_unvisited_target = None
            min_dist_to_unvisited = float('inf')

            # 确保对 unvisited_target_points 的遍历顺序是确定的
            sorted_unvisited_points = sorted(list(unvisited_target_points))

            for p in sorted_unvisited_points:
                dist_path = bfs_shortest_path_with_cache(
                    current_pos, p, grid_size, forbidden_nodes)
                if dist_path is not None:
                    dist = len(dist_path) - 1
                    if dist < min_dist_to_unvisited:
                        min_dist_to_unvisited = dist
                        closest_unvisited_target = p
                    elif dist == min_dist_to_unvisited:  # 字典序决胜规则
                        if closest_unvisited_target is None or p < closest_unvisited_target:
                            closest_unvisited_target = p

            if closest_unvisited_target is None:
                return [], float('inf'), False

            next_actual_target = closest_unvisited_target

        segment_path = bfs_shortest_path_with_cache(
            current_pos, next_actual_target, grid_size, forbidden_nodes)

        if segment_path is None:
            return [], float('inf'), False

        for node in segment_path[1:]:
            final_path.append(node)
            if node in unvisited_target_points:
                visited_in_final_path.add(node)
                unvisited_target_points.discard(node)

        current_pos = next_actual_target

    if current_pos != start_node:
        return_path = bfs_shortest_path_with_cache(
            current_pos, start_node, grid_size, forbidden_nodes)
        if return_path is None:
            return [], float('inf'), False

        final_path.extend(return_path[1:])

    return final_path, len(final_path) - 1, True

# --- 可视化函数 ---


def visualize_path(grid_size, path, forbidden_nodes, reachable_points, layout_index, path_len_display):
    rows, cols = grid_size
    fig, ax = plt.subplots(figsize=(cols, rows))

    ax.set_xticks(np.arange(-0.5, cols, 1))
    ax.set_yticks(np.arange(-0.5, rows, 1))
    ax.grid(True, which='major', color='black', linestyle='-', linewidth=2)
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.invert_yaxis()

    for r, c in reachable_points:
        ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1,
                                   facecolor='lightblue', edgecolor='blue', linewidth=0.5, alpha=0.5))

    for f_r, f_c in forbidden_nodes:
        ax.add_patch(plt.Rectangle((f_c - 0.5, f_r - 0.5), 1, 1,
                                   facecolor='gray', edgecolor='black', linewidth=1))
        ax.text(f_c, f_r, 'X',
                ha='center', va='center', color='white', fontsize=min(rows, cols)*2, fontweight='bold')

    if path and len(path) > 1:
        for i in range(len(path) - 1):
            p1 = path[i]
            p2 = path[i+1]

            x1, y1 = p1[1], p1[0]
            x2, y2 = p2[1], p2[0]

            ax.annotate("", xy=(x2, y2), xytext=(x1, y1),
                        arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0",
                                        color='purple', lw=1.5, shrinkA=5, shrinkB=5))

        visited_coords_on_path = set()
        for r, c in path:
            if (r, c) not in visited_coords_on_path:
                ax.plot(c, r, marker='o', markersize=3, color='yellow',
                        markeredgecolor='red', alpha=0.8)
                visited_coords_on_path.add((r, c))

        start_r, start_c = path[0]
        ax.text(start_c, start_r, 'S/E', ha='center', va='center', color='darkgreen', fontsize=16,
                bbox=dict(boxstyle="square,pad=0.2", fc="lightgreen", ec="green", lw=1))

    ax.set_title(f"布局 {layout_index}: 路径长度 {path_len_display}")
    plt.show()


# --- 主程序 ---
if __name__ == "__main__":
    grid_rows = 7
    grid_cols = 9
    num_forbidden_rects = 2  # 两个禁止区域
    # 修正：允许的禁止区域形状列表
    forbidden_rect_shapes = ((1, 3), (3, 1))
    start_point = (0, 0)

    # 生成所有可能的两个不重叠的 1x3 或 3x1 禁止区域组合
    all_forbidden_layouts = get_all_forbidden_layouts(
        grid_rows, grid_cols, num_forbidden_rects, forbidden_rect_shapes, start_point
    )
    print(f"找到 {len(all_forbidden_layouts)} 种不同的禁止区域摆放方式（两个不重叠的1x3或3x1矩形）。")

    results = {}

    total_start_time = time.time()

    for i, forbidden_layout in enumerate(all_forbidden_layouts):
        # 尽管 get_all_forbidden_layouts 已经过滤了包含起点的布局，这里可以再次确认
        if start_point in forbidden_layout:
            print(
                f"\n跳过布局 {i+1}/{len(all_forbidden_layouts)} {forbidden_layout}: 起点被禁止（应已被过滤）。")
            results[forbidden_layout] = (None, float('inf'))
            continue

        all_reachable_points_for_layout = []
        for r in range(grid_rows):
            # 修正：确保这里使用 grid_cols
            for c in range(grid_cols):
                if (r, c) not in forbidden_layout:
                    all_reachable_points_for_layout.append((r, c))

        is_connected = True
        if all_reachable_points_for_layout:
            # 在连通性检查前清空缓存，确保独立性
            BFS_CACHE = {}
            for p_target in all_reachable_points_for_layout:
                if p_target != start_point:
                    path_to_target = bfs_shortest_path_with_cache(
                        start_point, p_target, (grid_rows, grid_cols), forbidden_layout)
                    if path_to_target is None:
                        is_connected = False
                        break
        else:
            is_connected = False

        if not is_connected:
            print(
                f"\n跳过布局 {i+1}/{len(all_forbidden_layouts)} {forbidden_layout}: 某些可访问区域与起点不连通。")
            results[forbidden_layout] = (None, float('inf'))
            continue

        print(
            f"\n--- 正在计算布局 {i+1}/{len(all_forbidden_layouts)}: {forbidden_layout} ---")

        path, path_len, success = solve_grid_path_with_refinement(
            (grid_rows, grid_cols), forbidden_layout, start_point
        )

        results[forbidden_layout] = (path, path_len)

        if success:
            print(f"找到路径，长度: {path_len}")
            current_reachable_points = set()
            for r in range(grid_rows):
                for c in range(grid_cols):  # 再次确保这里是 grid_cols
                    if (r, c) not in forbidden_layout:
                        current_reachable_points.add((r, c))
            visualize_path((grid_rows, grid_cols), path, forbidden_layout,
                           current_reachable_points, i + 1, path_len)
        else:
            print("未能找到可行路径。")

    total_end_time = time.time()
    print(f"\n--- 所有布局计算完成 ---")
    print(f"总耗时: {total_end_time - total_start_time:.2f} 秒")

    print("\n--- 结果概览 ---")
    shortest_overall_len = float('inf')
    shortest_overall_layout = None

    for forbidden_layout, (path, path_len) in results.items():
        print(
            f"禁止区域: {forbidden_layout}, 路径长度: {path_len if path_len != float('inf') else 'N/A'}")
        if path_len < shortest_overall_len:
            shortest_overall_len = path_len
            shortest_overall_layout = forbidden_layout

    if shortest_overall_layout and shortest_overall_len != float('inf'):
        print(f"\n所有布局中最短路径长度: {shortest_overall_len}")
        print(f"对应的禁止区域: {shortest_overall_layout}")
    else:
        print("\n没有找到任何布局下的可行路径。")

    # 获取并打印当前新加坡时间
    current_singapore_time = time.strftime(
        "%Y-%m-%d %H:%M:%S", time.localtime(time.time() + (8 * 3600)))
    print(f"\n（当前新加坡时间: {current_singapore_time}）")
