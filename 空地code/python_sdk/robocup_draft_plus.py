import numpy as np
import cv2
import os

# --- 1. 检查并加载矩阵 (高精度) ---
H_PRIME_FILE = 'H_prime_LS_matrix_new.npy'
H_PRIME_MATRIX = None  # 初始化为 None

try:
    if not os.path.exists(H_PRIME_FILE):
        # 如果文件不存在，抛出错误，但我们会捕获并礼貌地提示用户
        raise FileNotFoundError(f"错误：未找到透视矩阵文件 '{H_PRIME_FILE}'。请先运行第一个代码文件。")

    # 加载高精度二进制矩阵
    H_PRIME_MATRIX = np.load(H_PRIME_FILE)

except FileNotFoundError as e:
    # 打印错误信息
    print(e)


def transform_dxdy_to_fxfy_homography(dx: float, dy: float) -> tuple[float, float]:
    """
    使用 Homography (最适合外推的物理模型) 将图像坐标 (dx, dy) 转换为现实坐标 (f_x, f_y)。
    """

    # 检查矩阵是否加载成功
    if H_PRIME_MATRIX is None:
        # 如果矩阵为空，安全退出，避免程序崩溃
        raise RuntimeError("模型矩阵未加载，无法执行转换。")

    # 输入点必须是 (1, 1, 2) 的 float64 数组，这是 cv2.perspectiveTransform 的要求
    image_point = np.array([[[dx, dy]]], dtype=np.float64)

    # 执行透视变换: H' 将 (dx, dy) 转换为 (f_y, f_x)
    world_point_transformed = cv2.perspectiveTransform(
        image_point, H_PRIME_MATRIX)

    # 提取结果
    fy = world_point_transformed[0, 0, 0]  # f_y 在索引 0
    fx = world_point_transformed[0, 0, 1]  # f_x 在索引 1

    # 返回用户要求的 (f_x, f_y) 顺序
    return fx, fy


# --- 2. 交互式调用 ---
if __name__ == '__main__':
    if H_PRIME_MATRIX is None:
        print("\n**请先运行 'train_homography_ls_new2.py' 文件生成模型矩阵。**")
    else:
        print("--- 坐标转换工具 (Homography 模型) ---")
        print(f"模型文件已加载: {H_PRIME_FILE}")
        print("输入图像坐标 (dx, dy) 进行转换。输入 'q' 退出。")

        while True:
            try:
                # 获取 dx 输入
                dx_input = input("请输入 dx (图像 X 坐标): ")
                if dx_input.lower() == 'q':
                    break
                dx = float(dx_input)

                # 获取 dy 输入
                dy_input = input("请输入 dy (图像 Y 坐标): ")
                if dy_input.lower() == 'q':
                    break
                dy = float(dy_input)

                # 执行转换
                fx, fy = transform_dxdy_to_fxfy_homography(dx, dy)

                # 输出结果
                print(f"\n[结果] 输入 (dx={dx:.2f}, dy={dy:.2f})")
                print(f"       -> 世界坐标 (f_x, f_y): ({fx:.4f} m, {fy:.4f} m)")
                print("-" * 30)

            except ValueError:
                print("输入无效！请确保输入的是有效的数字。")
            except Exception as e:
                # 捕获其他运行时错误
                print(f"发生错误: {e}")
                break

        print("程序已退出。")
