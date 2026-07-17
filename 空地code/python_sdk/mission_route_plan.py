import numpy as np
import collections
import matplotlib.pyplot as plt
import time
import sys
import random


class DroneMissionPlanner:
    """
    一个用于规划无人机在网格地图上任务路径的类。
    支持指定1x3或3x1的禁止区域，并优化了返航路径和障碍物绕行逻辑。
    根据蛇形方向改变快速返航通道的方向，保持它们垂直，并保证先走快速通道最远离起点的点。
    新增优化：如果障碍物有一部分在快速通道上，那么远离起点的被障碍物挡住的部分，就不再设为快速通道。
    现在支持多个1x3或3x1的禁止区域。
    """

    def __init__(self, rows, cols):
        """
        初始化任务规划器。

        参数:
            rows (int): 网格的行数。
            cols (int): 网格的列数。
        """
        if not (isinstance(rows, int) and rows > 0 and
                isinstance(cols, int) and cols > 0):
            raise ValueError("网格的行数和列数必须是正整数。")

        # 对于快速通道的预留，至少需要2行或2列
        if cols <= 1 and rows <= 1:
            raise ValueError("网格的行数和列数至少有一个必须大于1，以便预留返航通道。")

        self.rows = rows
        self.cols = cols
        self.grid_size = (rows, cols)
        self.bfs_cache = {}
        # 返航通道的索引和方向将根据蛇形方向动态确定
        self.return_lane_idx = None  # 可能是行索引或列索引
        self.return_lane_direction = None  # 'horizontal' (行) 或 'vertical' (列)
        self.effective_return_lane_points = set()  # 实际用于快速通道的点集

    def _bfs_shortest_path_with_cache(self, start_coords, target_coords, forbidden_nodes):
        """
        使用BFS找到从start_coords到target_coords的最短路径列表。
        使用实例缓存 self.bfs_cache 避免重复计算。
        """
        # 注意：forbidden_nodes 是一个 frozenset，因此可以作为字典的key
        forbidden_key = frozenset(forbidden_nodes)
        cache_key = (start_coords, target_coords, forbidden_key)

        if cache_key in self.bfs_cache:
            return self.bfs_cache[cache_key]

        # 尝试使用反向路径缓存
        reverse_cache_key = (target_coords, start_coords, forbidden_key)
        if reverse_cache_key in self.bfs_cache:
            path = self.bfs_cache[reverse_cache_key]
            if path:
                self.bfs_cache[cache_key] = path[::-1]
                return path[::-1]
            else:
                return None

        q = collections.deque([(start_coords, [start_coords])])
        visited = {start_coords}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while q:
            (r, c), path = q.popleft()

            if (r, c) == target_coords:
                self.bfs_cache[cache_key] = path
                return path

            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                next_node_coords = (nr, nc)

                if 0 <= nr < self.rows and 0 <= nc < self.cols and \
                   next_node_coords not in forbidden_nodes and \
                   next_node_coords not in visited:

                    visited.add(next_node_coords)
                    q.append((next_node_coords, path + [next_node_coords]))

        self.bfs_cache[cache_key] = None
        return None

    def _generate_boustrophedon_base_points(self, boustrophedon_direction):
        """
        生成完整的蛇形遍历基础点，不排除任何通道点，因为排除逻辑将在 _solve_grid_path_with_refinement 中动态处理。
        """
        path = []
        if boustrophedon_direction == 'horizontal':
            for r in range(self.rows):
                if r % 2 == 0:  # 偶数行：从左到右
                    for c in range(self.cols):
                        path.append((r, c))
                else:  # 奇数行：从右到左
                    for c in reversed(range(self.cols)):
                        path.append((r, c))
        else:  # vertical
            for c_idx, c in enumerate(range(self.cols)):
                if c_idx % 2 == 0:  # 偶数列：从上到下
                    for r in range(self.rows):
                        path.append((r, c))
                else:  # 奇数列：从下到上
                    for r in reversed(range(self.rows)):
                        path.append((r, c))
        return path

    def _solve_grid_path_with_refinement(self, all_forbidden_nodes=frozenset(), start_node=(0, 0), boustrophedon_direction='vertical'):
        """
        基于预设的蛇形遍历路径，规划完整的遍历路径。
        优先局部绕行，但允许回头以保证一定有路径输出。

        参数:
            all_forbidden_nodes (frozenset): 包含所有禁止区域坐标的集合。
            start_node (tuple): 起始点的 (row, col) 坐标。
            boustrophedon_direction (str): 'horizontal' 或 'vertical'，指示蛇形遍历方向。
        """
        self.bfs_cache = {}
        self.effective_return_lane_points = set()  # 每次规划前重置

        # 1. 动态确定返航通道的初始方向和位置 (例如，第0行或第0列)
        initial_return_lane_points = set()
        if boustrophedon_direction == 'vertical':  # 主扫描是垂直的，返航通道应该是水平的
            self.return_lane_direction = 'horizontal'
            self.return_lane_idx = 0  # 最上侧行作为水平通道
            if self.rows == 1:
                print("网格行数为1，无法为垂直蛇形预留水平返航通道，将不使用快速通道。")
            else:
                for c in range(self.cols):
                    initial_return_lane_points.add((self.return_lane_idx, c))
        elif boustrophedon_direction == 'horizontal':  # 主扫描是水平的，返航通道应该是垂直的
            self.return_lane_direction = 'vertical'
            self.return_lane_idx = 0  # 最左侧列作为垂直通道
            if self.cols == 1:
                print("网格列数为1，无法为水平蛇形预留垂直返航通道，将不使用快速通道。")
            else:
                for r in range(self.rows):
                    initial_return_lane_points.add((r, self.return_lane_idx))
        else:
            raise ValueError(
                "boustrophedon_direction 必须是 'horizontal' 或 'vertical'。"
            )

        # 2. 根据障碍物裁剪快速通道
        valid_initial_lane_points = {
            p for p in initial_return_lane_points if p not in all_forbidden_nodes}
        forbidden_on_lane = initial_return_lane_points.intersection(
            all_forbidden_nodes)

        if not forbidden_on_lane:
            self.effective_return_lane_points = valid_initial_lane_points
            # print("快速通道上没有障碍物，将使用完整快速通道。")
        else:
            farthest_obstacle_on_lane = None
            max_dist_to_obstacle = -1

            for obs_p in forbidden_on_lane:
                if self.return_lane_direction == 'vertical':  # 垂直通道，沿行号判断距离起点
                    dist = abs(obs_p[0] - start_node[0])
                else:  # 水平通道，沿列号判断距离起点
                    dist = abs(obs_p[1] - start_node[1])

                if dist > max_dist_to_obstacle:
                    max_dist_to_obstacle = dist
                    farthest_obstacle_on_lane = obs_p
                elif dist == max_dist_to_obstacle:  # 距离相同时，选择坐标较小的点作为最远障碍物
                    if self.return_lane_direction == 'vertical':
                        if obs_p[0] < farthest_obstacle_on_lane[0]:
                            farthest_obstacle_on_lane = obs_p
                    else:
                        if obs_p[1] < farthest_obstacle_on_lane[1]:
                            farthest_obstacle_on_lane = obs_p

            if farthest_obstacle_on_lane:
                for p in valid_initial_lane_points:
                    # 检查点 p 是否在起点到最远障碍物的“安全”一侧
                    if self.return_lane_direction == 'vertical':  # 对于垂直通道，沿行号判断
                        if (start_node[0] <= farthest_obstacle_on_lane[0] and p[0] <= farthest_obstacle_on_lane[0]) or \
                           (start_node[0] > farthest_obstacle_on_lane[0] and p[0] >= farthest_obstacle_on_lane[0]):
                            self.effective_return_lane_points.add(p)
                    else:  # horizontal 对于水平通道，沿列号判断
                        if (start_node[1] <= farthest_obstacle_on_lane[1] and p[1] <= farthest_obstacle_on_lane[1]) or \
                           (start_node[1] > farthest_obstacle_on_lane[1] and p[1] >= farthest_obstacle_on_lane[1]):
                            self.effective_return_lane_points.add(p)
                # print(f"快速通道被障碍物 {farthest_obstacle_on_lane} 裁剪，实际快速通道点数：{len(self.effective_return_lane_points)}")
            else:
                self.effective_return_lane_points = valid_initial_lane_points

        # 3. 确定所有可达点
        all_reachable_points_set = set()
        for r in range(self.rows):
            for c in range(self.cols):
                if (r, c) not in all_forbidden_nodes:
                    all_reachable_points_set.add((r, c))

        if not all_reachable_points_set or (start_node not in all_reachable_points_set):
            # print("网格中没有可达点或起点不可达。")
            return [], float('inf'), False

        final_path = []
        current_pos = start_node
        final_path.append(start_node)

        unvisited_target_points = all_reachable_points_set.copy()
        unvisited_target_points.discard(start_node)

        # 定义主遍历需要覆盖的点 (排除实际快速通道中的点和起点)
        points_to_cover_by_boustrophedon = set()
        for p in all_reachable_points_set:
            if p == start_node:
                continue
            if p in self.effective_return_lane_points:
                continue
            points_to_cover_by_boustrophedon.add(p)

        full_boustrophedon_pattern = self._generate_boustrophedon_base_points(
            boustrophedon_direction)

        # 筛选出实际需要通过主遍历覆盖的点，并保持其原始蛇形顺序
        original_base_path_targets_ordered = []
        for p in full_boustrophedon_pattern:
            if p not in all_forbidden_nodes and p in points_to_cover_by_boustrophedon:
                original_base_path_targets_ordered.append(p)

        base_path_progress_idx = 0

        # --- 阶段1: 覆盖非返航通道区域 (优先遵循蛇形逻辑，必要时绕行或跳跃) ---
        while points_to_cover_by_boustrophedon:
            potential_next_logical_target = None
            # 找到原始蛇形路径中第一个未访问的逻辑目标点
            while base_path_progress_idx < len(original_base_path_targets_ordered):
                candidate_logical_target = original_base_path_targets_ordered[
                    base_path_progress_idx]
                if candidate_logical_target in points_to_cover_by_boustrophedon:
                    potential_next_logical_target = candidate_logical_target
                    break
                else:
                    base_path_progress_idx += 1  # 跳过已被访问的点

            next_actual_step = None
            segment_path = None  # 用于存储 BFS 找到的路径片段

            # 优先级1: 当前位置就是逻辑目标点
            if current_pos == potential_next_logical_target:
                next_actual_step = potential_next_logical_target
                segment_path = [current_pos]  # 自行处理，无需BFS
                base_path_progress_idx += 1  # 推进逻辑进度

            # 优先级2: 尝试通过BFS到达逻辑目标点
            if next_actual_step is None and potential_next_logical_target:
                path_to_logical_target = self._bfs_shortest_path_with_cache(
                    current_pos, potential_next_logical_target, all_forbidden_nodes
                )
                # 确保路径存在且目标点不是当前点
                if path_to_logical_target and len(path_to_logical_target) > 1:
                    next_actual_step = potential_next_logical_target
                    segment_path = path_to_logical_target
                    # 更新逻辑进度，跳过所有原始蛇形路径中到此目标点之前的点
                    actual_step_idx = original_base_path_targets_ordered.index(
                        next_actual_step)
                    base_path_progress_idx = max(
                        base_path_progress_idx, actual_step_idx + 1)

            # 优先级3: 如果逻辑目标不可达或没有逻辑目标，则进行局部探索 (寻找相邻的未访问点)
            if next_actual_step is None:
                candidate_neighbors = []
                directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
                for dr, dc in directions:
                    nr, nc = current_pos[0] + dr, current_pos[1] + dc
                    neighbor_node = (nr, nc)
                    if 0 <= nr < self.rows and 0 <= nc < self.cols and \
                       neighbor_node not in all_forbidden_nodes and \
                       neighbor_node in points_to_cover_by_boustrophedon:  # 必须是未访问的待覆盖点
                        candidate_neighbors.append(neighbor_node)

                if candidate_neighbors:
                    best_local_candidate = None
                    best_score = -float('inf')

                    for neighbor_p in candidate_neighbors:
                        score = 0
                        # 强力偏好：保持当前蛇形方向 (高分)
                        if boustrophedon_direction == 'horizontal':
                            if current_pos[0] % 2 == 0:  # 偶数行：从左到右
                                if neighbor_p[1] > current_pos[1]:
                                    score += 100  # 优先向右
                            else:  # 奇数行：从右到左
                                if neighbor_p[1] < current_pos[1]:
                                    score += 100  # 优先向左
                            # 次级偏好：垂直移动（换行），避免直接回头到已访问行
                            if neighbor_p[0] != current_pos[0]:
                                if neighbor_p[0] > current_pos[0]:  # 倾向于向下切换行
                                    score += 50
                                else:  # 向上切换行
                                    score += 25
                        else:  # vertical
                            current_col_idx_in_boustrophedon = current_pos[1]
                            if current_col_idx_in_boustrophedon % 2 == 0:  # 偶数列：从上到下
                                if neighbor_p[0] > current_pos[0]:
                                    score += 100
                            else:  # 奇数列：从下到上
                                if neighbor_p[0] < current_pos[0]:
                                    score += 100
                            # 次级偏好：水平移动（换列），避免直接回头到已访问列
                            if neighbor_p[1] != current_pos[1]:
                                if neighbor_p[1] > current_pos[1]:  # 倾向于向右切换列
                                    score += 50
                                else:  # 向左切换列
                                    score += 25

                        # 考虑与原始蛇形路径的“下一个逻辑目标点”的曼哈顿距离。（作为 tie-breaker）
                        if potential_next_logical_target:
                            dist_to_logical_target = abs(neighbor_p[0] - potential_next_logical_target[0]) + abs(
                                neighbor_p[1] - potential_next_logical_target[1])
                            score -= dist_to_logical_target  # 距离越远，扣分越多

                        if score > best_score:
                            best_score = score
                            best_local_candidate = neighbor_p
                        elif score == best_score:
                            if best_local_candidate is None or neighbor_p < best_local_candidate:  # 坐标小的优先级更高
                                best_local_candidate = neighbor_p

                    next_actual_step = best_local_candidate
                    segment_path = [
                        current_pos, next_actual_step] if next_actual_step else None

                    # 即使通过局部绕行，也要尝试更新 base_path_progress_idx，确保整体进度
                    if next_actual_step and next_actual_step in original_base_path_targets_ordered:
                        actual_step_idx = original_base_path_targets_ordered.index(
                            next_actual_step)
                        base_path_progress_idx = max(
                            base_path_progress_idx, actual_step_idx + 1)
                    elif next_actual_step:  # 如果是绕行到了不在逻辑路径上的点，至少推进一步
                        base_path_progress_idx += 1

            # 优先级4: 如果局部探索也无法找到合适的下一步（被困或无法保持连续性），则进行全局跳跃
            if next_actual_step is None:
                # 寻找所有剩余待覆盖点中，距离当前位置最近且可达的点
                closest_unvisited_target = None
                min_dist_to_unvisited = float('inf')

                sorted_unvisited_points = sorted(
                    list(points_to_cover_by_boustrophedon))

                for p in sorted_unvisited_points:
                    path_to_p = self._bfs_shortest_path_with_cache(
                        current_pos, p, all_forbidden_nodes)
                    if path_to_p is not None:
                        dist = len(path_to_p) - 1  # 路径长度作为距离
                        if dist < min_dist_to_unvisited:
                            min_dist_to_unvisited = dist
                            closest_unvisited_target = p
                        elif dist == min_dist_to_unvisited:
                            if closest_unvisited_target is None or p < closest_unvisited_target:
                                closest_unvisited_target = p

                if closest_unvisited_target is None:
                    # print(f"错误：阶段1无法从 {current_pos} 到达任何剩余的待覆盖点。可能存在无法遍历的区域。")
                    return [], float('inf'), False  # 致命错误，无法覆盖所有点

                next_actual_step = closest_unvisited_target
                segment_path = self._bfs_shortest_path_with_cache(
                    current_pos, next_actual_step, all_forbidden_nodes)

                # 跳跃后，更新 base_path_progress_idx，跳过中间点以保证进度
                if next_actual_step in original_base_path_targets_ordered:
                    actual_step_idx = original_base_path_targets_ordered.index(
                        next_actual_step)
                    base_path_progress_idx = max(
                        base_path_progress_idx, actual_step_idx + 1)
                else:
                    base_path_progress_idx += 1  # 确保向前推进

            if segment_path is None:  # 这种情况理论上不应该发生，但为了鲁棒性检查
                # print(f"错误：无法从 {current_pos} 找到任何到 {next_actual_step} 的路径。")
                return [], float('inf'), False

            for node in segment_path[1:]:  # 路径的第一点是current_pos，跳过
                final_path.append(node)
                if node in points_to_cover_by_boustrophedon:
                    points_to_cover_by_boustrophedon.discard(node)
                if node in unvisited_target_points:
                    unvisited_target_points.discard(node)

            current_pos = next_actual_step

        # --- 阶段2: 移动到实际的快速通道并访问通道内的点 (从最远离起点的点开始) ---
        return_lane_points_unvisited = []
        for p in self.effective_return_lane_points:
            if p in unvisited_target_points:  # 确保只考虑未访问的通道点
                return_lane_points_unvisited.append(p)

        if return_lane_points_unvisited:
            # 找到距离起点最远的点（在 effective_return_lane_points 中）
            farthest_return_lane_point = None
            max_dist_to_start = -1

            for p in return_lane_points_unvisited:
                if self.return_lane_direction == 'vertical':
                    dist_from_start_axis = abs(p[0] - start_node[0])
                else:
                    dist_from_start_axis = abs(p[1] - start_node[1])

                if dist_from_start_axis > max_dist_to_start:
                    max_dist_to_start = dist_from_start_axis
                    farthest_return_lane_point = p
                elif dist_from_start_axis == max_dist_to_start:  # 距离相同时，选择坐标较小的点
                    if self.return_lane_direction == 'vertical':
                        if p[0] < farthest_return_lane_point[0]:
                            farthest_return_lane_point = p
                    else:
                        if p[1] < farthest_return_lane_point[1]:
                            farthest_return_lane_point = p

            if farthest_return_lane_point:
                segment_path_to_farthest = self._bfs_shortest_path_with_cache(
                    current_pos, farthest_return_lane_point, all_forbidden_nodes)
                if segment_path_to_farthest:
                    for node in segment_path_to_farthest[1:]:
                        final_path.append(node)
                        if node in unvisited_target_points:
                            unvisited_target_points.discard(node)
                    current_pos = farthest_return_lane_point
                else:
                    # print(
                    # f"警告：无法从 {current_pos} 到达最远的有效返航通道点 {farthest_return_lane_point}。")
                    return [], float('inf'), False

                # 从最远点开始，沿着通道向起点遍历
                return_lane_points_unvisited = [
                    p for p in self.effective_return_lane_points if p in unvisited_target_points]

                # 对未访问的返航通道点进行排序，以确保从最远点向起点方向遍历
                if self.return_lane_direction == 'vertical':  # 垂直通道，从上到下或从下到上
                    # 如果起点在通道上方（行号较小），则从大行号向小行号遍历
                    if start_node[0] < farthest_return_lane_point[0]:
                        return_lane_points_unvisited.sort(
                            key=lambda p: p[0], reverse=True)
                    else:  # 起点在通道下方（行号较大），则从小行号向大行号遍历
                        return_lane_points_unvisited.sort(key=lambda p: p[0])
                else:  # horizontal 水平通道，从左到右或从右到左
                    # 如果起点在通道左侧（列号较小），则从大列号向小列号遍历
                    if start_node[1] < farthest_return_lane_point[1]:
                        return_lane_points_unvisited.sort(
                            key=lambda p: p[1], reverse=True)
                    else:  # 起点在通道右侧（列号较大），则从小列号向大列号遍历
                        return_lane_points_unvisited.sort(key=lambda p: p[1])

            for target_p in return_lane_points_unvisited:
                if target_p not in unvisited_target_points:
                    continue  # 已经访问过了

                if current_pos == target_p:
                    continue  # 已经是当前点

                segment_path = self._bfs_shortest_path_with_cache(
                    current_pos, target_p, all_forbidden_nodes)
                if segment_path:
                    for node in segment_path[1:]:
                        final_path.append(node)
                        if node in unvisited_target_points:
                            unvisited_target_points.discard(node)
                    current_pos = target_p
                else:
                    # print(f"警告：在有效返航通道中无法从 {current_pos} 到达 {target_p}。")
                    pass  # 允许快速通道中有无法到达的点，这些点会被主遍历或跳跃覆盖

        # --- 阶段3: 返回起点 ---
        if unvisited_target_points:
            # print(f"错误：仍有未访问的可达点：{unvisited_target_points}")
            # 理论上不应该到这里，因为所有点都应该被覆盖
            return [], float('inf'), False

        if current_pos != start_node:
            return_path_to_start = self._bfs_shortest_path_with_cache(
                current_pos, start_node, all_forbidden_nodes)
            if return_path_to_start is None:
                # print(f"警告：无法从 {current_pos} 返回到起点 {start_node}。")
                return [], float('inf'), False
            final_path.extend(return_path_to_start[1:])

        return final_path, len(final_path) - 1, True

    def _validate_single_forbidden_block(self, block_coords):
        """
        验证单个禁止区域坐标是否构成有效的 1x3 或 3x1 矩形。
        返回其形状信息。
        """
        block_set = frozenset(block_coords)
        num_nodes = 3

        if len(block_set) != num_nodes:
            raise ValueError(
                f"禁止区域块的节点数量必须是 {num_nodes}，但传入了 {len(block_set)} 个：{block_coords}。")

        for r, c in block_set:
            if not (0 <= r < self.rows and 0 <= c < self.cols):
                raise ValueError(f"禁止区域坐标 ({r}, {c}) 超出网格范围 {self.grid_size}。")

        min_r = min(p[0] for p in block_set)
        max_r = max(p[0] for p in block_set)
        min_c = min(p[1] for p in block_set)
        max_c = max(p[1] for p in block_set)

        height = max_r - min_r + 1
        width = max_c - min_c + 1

        is_1x3 = (height == 1 and width == num_nodes)
        is_3x1 = (height == num_nodes and width == 1)

        if not (is_1x3 or is_3x1):
            raise ValueError(
                f"禁止区域块 {block_coords} 未构成有效的 {num_nodes} 格 1x{num_nodes} 或 {num_nodes}x1 矩形。")

        expected_cells = set()
        for r_offset in range(height):
            for c_offset in range(width):
                expected_cells.add((min_r + r_offset, min_c + c_offset))

        if expected_cells != block_set:
            raise ValueError(f"禁止区域块 {block_coords} 构成的形状不连续或不符合矩形预期。")

        if is_1x3:
            return 'horizontal'
        else:
            return 'vertical'

    def _visualize_path(self, path, forbidden_nodes, reachable_points, layout_description, path_len_display):
        """
        可视化网格和规划的路径。
        """
        fig, ax = plt.subplots(figsize=(self.cols, self.rows))

        ax.set_xticks(np.arange(-0.5, self.cols, 1))
        ax.set_yticks(np.arange(-0.5, self.rows, 1))
        ax.grid(True, which='major', color='black', linestyle='-', linewidth=2)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.invert_yaxis()

        for r, c in reachable_points:
            ax.add_patch(plt.Rectangle((c - 0.5, r - 0.5), 1, 1,
                                       facecolor='lightblue', edgecolor='blue', linewidth=0.5, alpha=0.5))

        # 绘制实际的快速返航通道
        for r_lane, c_lane in self.effective_return_lane_points:
            ax.add_patch(plt.Rectangle((c_lane - 0.5, r_lane - 0.5), 1, 1,
                                       facecolor='lightgreen', edgecolor='green', linewidth=0.5, alpha=0.5))

        for f_r, f_c in forbidden_nodes:
            ax.add_patch(plt.Rectangle((f_c - 0.5, f_r - 0.5), 1, 1,
                                       facecolor='gray', edgecolor='black', linewidth=1))
            ax.text(f_c, f_r, 'X',
                    ha='center', va='center', color='white', fontsize=min(self.rows, self.cols)*2, fontweight='bold')

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
                    ax.plot(c, r, marker='o', markersize=3,
                            color='yellow', markeredgecolor='red', alpha=0.8)
                    visited_coords_on_path.add((r, c))

            start_r, start_c = path[0]
            ax.text(start_c, start_r, 'S/E', ha='center', va='center', color='darkgreen', fontsize=16,
                    bbox=dict(boxstyle="square,pad=0.2", fc="lightgreen", ec="green", lw=1))

        ax.set_title(f"{layout_description}: 路径长度 {path_len_display}")
        plt.show()

    def plan_and_visualize_mission(self, list_of_forbidden_blocks, start_point=(0, 0)):
        """
        规划并可视化无人机任务路径。这是外部调用的主要接口。
        现在支持多个 1x3 或 3x1 的禁止区域。
        将尝试水平和垂直两种蛇形遍历方向，并选择路径最短的方案。

        参数:
            list_of_forbidden_blocks (list): 一个列表，每个元素是一个包含 3 个坐标元组的列表/集合，代表一个 1x3 或 3x1 的禁止区域块。
                                             例如：[[(0,0), (0,1), (0,2)], [(3,3), (4,3), (5,3)]]
            start_point (tuple): 起始点的 (row, col) 坐标，默认为 (0,0)。

        返回:
            list: 如果成功找到路径，返回路径的坐标列表；否则返回空列表。
        """
        if not (0 <= start_point[0] < self.rows and 0 <= start_point[1] < self.cols):
            raise ValueError(
                f"起点 {start_point} 超出网格范围 {self.grid_size}。"
            )

        all_forbidden_nodes = set()
        for i, block_coords in enumerate(list_of_forbidden_blocks):
            try:
                self._validate_single_forbidden_block(block_coords)
                block_set = frozenset(block_coords)
                if all_forbidden_nodes.intersection(block_set):
                    raise ValueError(f"禁止区域块 {block_coords} 与其他禁止区域重叠。")
                all_forbidden_nodes.update(block_set)
                print(f"禁止区域块 {i+1} 已验证：{block_coords}")
            except ValueError as e:
                raise ValueError(f"禁止区域块 {i+1} 验证失败：{e}") from e

        all_forbidden_nodes = frozenset(all_forbidden_nodes)

        if start_point in all_forbidden_nodes:
            raise ValueError(
                f"起点 {start_point} 位于禁止区域 {all_forbidden_nodes} 内，无法规划。")

        print(f"\n--- 正在计算路径 for 禁止区域: {all_forbidden_nodes} ---")

        best_path = []
        min_path_len = float('inf')
        best_direction = None
        best_return_lane_info = ""

        # 尝试水平蛇形遍历
        print("\n--- 尝试水平蛇形遍历 ---")
        temp_planner_h = DroneMissionPlanner(
            self.rows, self.cols)  # 使用临时planner，避免状态互相影响
        path_h, path_len_h, success_h = temp_planner_h._solve_grid_path_with_refinement(
            all_forbidden_nodes, start_point, 'horizontal'
        )
        if success_h:
            print(f"水平蛇形找到路径，长度: {path_len_h}")
            if path_len_h < min_path_len:
                min_path_len = path_len_h
                best_path = path_h
                best_direction = 'horizontal'
                best_return_lane_info = f"快速通道: {'Row' if temp_planner_h.return_lane_direction == 'horizontal' else 'Col'} {temp_planner_h.return_lane_idx} (裁剪后包含 {len(temp_planner_h.effective_return_lane_points)} 点)"

        # 尝试垂直蛇形遍历
        print("\n--- 尝试垂直蛇形遍历 ---")
        temp_planner_v = DroneMissionPlanner(
            self.rows, self.cols)  # 使用临时planner
        path_v, path_len_v, success_v = temp_planner_v._solve_grid_path_with_refinement(
            all_forbidden_nodes, start_point, 'vertical'
        )
        if success_v:
            print(f"垂直蛇形找到路径，长度: {path_len_v}")
            if path_len_v < min_path_len:
                min_path_len = path_len_v
                best_path = path_v
                best_direction = 'vertical'
                best_return_lane_info = f"快速通道: {'Row' if temp_planner_v.return_lane_direction == 'horizontal' else 'Col'} {temp_planner_v.return_lane_idx} (裁剪后包含 {len(temp_planner_v.effective_return_lane_points)} 点)"

        # 将最优方案的快速通道信息复制到当前planner，以便可视化
        if best_direction == 'horizontal':
            self.return_lane_idx = temp_planner_h.return_lane_idx
            self.return_lane_direction = temp_planner_h.return_lane_direction
            self.effective_return_lane_points = temp_planner_h.effective_return_lane_points
        elif best_direction == 'vertical':
            self.return_lane_idx = temp_planner_v.return_lane_idx
            self.return_lane_direction = temp_planner_v.return_lane_direction
            self.effective_return_lane_points = temp_planner_v.effective_return_lane_points

        if best_path:
            print(
                f"\n最终选择 {best_direction.capitalize()} 蛇形遍历，路径长度: {min_path_len}")
            current_reachable_points = set()
            for r in range(self.rows):
                for c in range(self.cols):
                    if (r, c) not in all_forbidden_nodes:
                        current_reachable_points.add((r, c))

            self._visualize_path(
                best_path,
                all_forbidden_nodes,
                current_reachable_points,
                f"{best_direction.capitalize()}蛇形 ({best_return_lane_info}), 禁区: {all_forbidden_nodes}",
                min_path_len
            )
            return best_path
        else:
            print("未能找到可行路径。")
            return []

# --- 辅助函数：生成随机禁止区域 ---


def generate_random_forbidden_area_list(rows, cols, start_point, num_obstacles=1, num_attempts_per_obstacle=100):
    """
    随机生成 num_obstacles 个 3x1 或 1x3 的禁止区域，确保它们在网格内、不与起点重叠，且彼此不重叠。
    """
    generated_forbidden_blocks = []
    all_forbidden_coords_set = set()

    for _ in range(num_obstacles):
        found_block = False
        for _attempt in range(num_attempts_per_obstacle):
            forbidden_coords_candidate = set()
            is_horizontal = random.choice([True, False])

            if is_horizontal:
                if cols < 3:
                    continue
                r = random.randrange(rows)
                c_start = random.randrange(cols - 2)
                for i in range(3):
                    forbidden_coords_candidate.add((r, c_start + i))
            else:
                if rows < 3:
                    continue
                r_start = random.randrange(rows - 2)
                c = random.randrange(cols)
                for i in range(3):
                    forbidden_coords_candidate.add((r_start + i, c))

            # 检查与起点和已生成的障碍物是否重叠
            if forbidden_coords_candidate.intersection({start_point}):
                continue
            if forbidden_coords_candidate.intersection(all_forbidden_coords_set):
                continue

            generated_forbidden_blocks.append(list(forbidden_coords_candidate))
            all_forbidden_coords_set.update(forbidden_coords_candidate)
            found_block = True
            break

        if not found_block:
            raise RuntimeError(
                f"无法在指定网格和起点下生成 {num_obstacles} 个不重叠的随机禁止区域。请检查网格大小或起点位置。")

    return generated_forbidden_blocks


# --- 主程序 ---
if __name__ == "__main__":
    grid_rows = 7
    grid_cols = 9
    start_point = (0, 0)

    planner = DroneMissionPlanner(grid_rows, grid_cols)

    num_random_scenarios = 5  # 减少场景数量以加快运行
    num_obstacles_per_scenario = 1  # 尝试生成2个障碍物

    for i in range(num_random_scenarios):
        print(f"\n" + "="*50)
        print(
            f"--- 随机场景 {i+1}/{num_random_scenarios} (生成 {num_obstacles_per_scenario} 个障碍物) ---")

        try:
            random_forbidden_blocks = generate_random_forbidden_area_list(
                grid_rows, grid_cols, start_point, num_obstacles=num_obstacles_per_scenario
            )
            print(f"生成的随机禁止区域块: {random_forbidden_blocks}")

            planned_path = planner.plan_and_visualize_mission(
                random_forbidden_blocks, start_point
            )
            if planned_path:
                print(f"{planned_path}")
                print(f"场景 {i+1} 规划成功，路径长度: {len(planned_path) - 1}")
            else:
                print(f"场景 {i+1} 规划失败（未找到路径）。")
        except ValueError as e:
            print(f"**场景 {i+1} 错误：{e}**")
        except RuntimeError as e:
            print(f"**场景 {i+1} 生成随机禁止区域错误：{e}**")

    current_singapore_time = time.strftime(
        "%Y-%m-%d %H:%M:%S", time.localtime(time.time() + (8 * 3600)))
    print(f"\n（当前新加坡时间: {current_singapore_time}）")
