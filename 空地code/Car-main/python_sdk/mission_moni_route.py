from SC import calc_spline_course
from loguru import logger
import numpy as np

routes = {  # 1:S>B,2:B>A,3:A>B,4:B>GB,5:A>GA,6:GB>S,7:GA>S
    1: (
        [0.25, 0.7777777777777777, 1.0694444444444446, 1.95],
        [0.25, 1.0952380952380953, 1.292517006802721, 1.45]),
    2: (
        [1.60, 1.90, 2.95, 2.99],
        [1.45, 1.45, 1.45, 1.45],
    ),
    3: (
        [2.50, 2.90, 3.5625, 3.7569444444444444, 3.7083333333333335, 3.2916666666666665,
         2.9166666666666665, 1.625, 1.3541666666666665, 1.3472222222222223, 1.5069444444444446, 1.95],
        [1.45, 1.45, 1.4421768707482993, 1.1564625850340136, 0.8979591836734695, 0.816326530612245,
         0.7891156462585034, 0.816326530612245, 0.9319727891156463, 1.1224489795918367, 1.360544217687075, 1.45],
    ),
    4: (
        [1.60, 1.90, 2.291666666666667, 2.3958333333333335,
         2.3472222222222223, 1.9444444444444444, 1.6],
        [1.45, 1.45, 1.4965986394557823, 1.7142857142857144,
         1.925170068027211, 1.9591836734693877, 1.95],
    ),
    5: (
        [2.50, 2.90, 3.4930555555555556, 3.5208333333333335,
         3.354166666666667, 3.006944444444444, 2.7],
        [1.45, 1.45, 1.5714285714285716, 1.8231292517006803,
         1.9863945578231293, 1.9931972789115646, 1.99],
    ),
    6: (
        [1.6, 1.90, 2.0208333333333335, 1.4444444444444446, 1.0902777777777777,
         0.6736111111111112, 0.22222222222222232, 0.14583333333333337,],
        [1.96, 1.96, 1.9659863945578233, 1.8979591360544217, 1.5578231292517007,
         0.9659863945578233, 0.1972789115646258, 0.10884353741496597],
    ),
    7: (
        [3.3, 3, 2.9930555555555554, 2.048611111111111, 1.451388888888889,
         0.8055555555555554, 0.25, 0.11805555555555547],
        [1.96, 1.96, 1.9727891156462585, 1.925170068027211, 1.7687074829931975,
         1.0612244897959183, 0.2312925170068025, 0.11564625850340134],
    ),
}


def validate_endpoint_separate(x_list, y_list, new_x, new_y, min_dist=0.1):
    """
    适配分离坐标列表的终点校验

    :param x_list: 历史x坐标 [x1, x2,...]
    :param y_list: 历史y坐标 [y1, y2,...]
    :param new_x: 待添加x坐标
    :param new_y: 待添加y坐标
    :param min_dist: 最小有效移动距离
    :return: bool (True=有效终点)
    """
    if len(x_list) != len(y_list):
        raise ValueError("x/y坐标列表长度不一致")
    if not x_list:
        return True
    last_x, last_y = x_list[-1], y_list[-1]
    # 检查移动距离
    move_dist = ((new_x - last_x)**2 + (new_y - last_y)**2)**0.5
    if move_dist < min_dist:
        print(f"警告：移动距离 {move_dist:.3f} < 阈值 {min_dist}")
        return False
    # 方向检查（需至少2个历史点）
    if len(x_list) >= 2:
        prev_x, prev_y = x_list[-2], y_list[-2]
        hist_vec_x, hist_vec_y = last_x - prev_x, last_y - prev_y
        new_vec_x, new_vec_y = new_x - last_x, new_y - last_y
        # 计算方向相似度
        dot_product = hist_vec_x * new_vec_x + hist_vec_y * new_vec_y
        hist_norm = (hist_vec_x**2 + hist_vec_y**2)**0.5
        new_norm = (new_vec_x**2 + new_vec_y**2)**0.5

        if hist_norm > 1e-6 and new_norm > 1e-6:
            cos_theta = dot_product / (hist_norm * new_norm)
            if cos_theta < 0:  # 90度
                print(f"警告：方向突变 (角度>{np.degrees(np.arccos(cos_theta)):.1f}°)")
                return False

    return True


def get_route(x, route_number, dl=0.1):
    route = routes[route_number]
    route_x, route_y = list(route[0]), list(route[1])
    # 计算route部分的样条曲线
    cx_route, cy_route, cyaw_route, ck_route, _ = calc_spline_course(
        route_x, route_y, ds=dl)

    # 查找route分割点
    split_idx_route = 0
    first_state_route = x < cx_route[split_idx_route]
    while split_idx_route < len(cx_route) - 1 and (first_state_route == (x < cx_route[split_idx_route])):
        split_idx_route += 1

    if split_idx_route == len(cx_route) - 1:
        split_idx_route = 0
        # 初始化返回参数
    route_params = [[], [], [], []]
    # 填充enter_params
    # for i, data in enumerate([cx_enter, cy_enter, cyaw_enter, ck_enter]):
    #     enter_params[i].extend(data[:split_idx_enter])
    # # 填充route_params
    # for i, data in enumerate([cx_route, cy_route, cyaw_route, ck_route]):
    #     route_params[i].extend(data[split_idx_route:])
    route_params[0].extend(cx_route[split_idx_route:])
    route_params[1].extend(cy_route[split_idx_route:])
    route_params[2].extend(cyaw_route[split_idx_route:])
    route_params[3].extend(ck_route[split_idx_route:])
    logger.debug(f"{route_params}")
    return route_params
