import cv2
import numpy as np
import os
from pathlib import Path


def apply_clahe(image_path, clip_limit=2.0, tile_grid_size=(8, 8), output_path=None):
    """
    对单张图像应用CLAHE（对比度受限的自适应直方图均衡化）算法。

    Args:
        image_path (str): 输入图像的完整路径。
        clip_limit (float): 对比度限制阈值。默认值通常在1.0到4.0之间。
                            值越大，对比度增强越强。
        tile_grid_size (tuple): 图像被分割成的小块网格大小(width, height)。
                                 较小的尺寸会导致更多的局部增强。
        output_path (str, optional): 增强后图像的保存路径。如果为None，则不保存。
                                     如果只提供目录，则按原文件名保存到该目录。

    Returns:
        numpy.ndarray: 增强后的图像（BGR格式），如果处理失败则返回None。
    """
    try:
        # 读取图像
        img = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if img is None:
            print(f"错误: 无法读取图像 '{image_path}'。请检查路径或文件完整性。")
            return None

        # 将图像转换为灰度图（CLAHE通常在灰度图上操作，然后应用到彩色图的V通道或Y通道）
        # 对于彩色图像，通常会转换为Lab或YCrCb色彩空间，在L或Y通道进行均衡化，再转回BGR。
        # 这里为了简单，我们先转换为灰度图进行演示。
        # 如果需要处理彩色图像的对比度，推荐转换到Lab空间对L通道处理。

        # 示例：处理灰度图
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 创建CLAHE对象
        clahe = cv2.createCLAHE(clipLimit=clip_limit,
                                tileGridSize=tile_grid_size)

        # 应用CLAHE
        enhanced_gray_img = clahe.apply(gray_img)

        # 如果需要输出彩色图像，可以将增强后的灰度图转换回彩色，或者对Lab空间的L通道进行处理
        # 这里我们将增强后的灰度图作为最终输出，或者如果您想保留颜色，可以这样做：
        # merged_img = cv2.cvtColor(np.stack([enhanced_gray_img, gray_img, gray_img], axis=2), cv2.COLOR_GRAY2BGR) # 示例，简单的灰度扩展到BGR
        # 或者更专业的Lab空间处理:
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l_channel = lab[:, :, 0]
        a_channel = lab[:, :, 1]
        b_channel = lab[:, :, 2]

        enhanced_l_channel = clahe.apply(l_channel)

        # 合并通道并转回BGR
        merged_lab = cv2.merge([enhanced_l_channel, a_channel, b_channel])
        enhanced_color_img = cv2.cvtColor(merged_lab, cv2.COLOR_LAB2BGR)

        # 保存增强后的图像
        if output_path:
            # 如果output_path是目录，则构建完整的文件路径
            if Path(output_path).is_dir():
                file_name = Path(image_path).name
                final_output_path = Path(output_path) / file_name
            else:
                final_output_path = Path(output_path)

            # 确保输出目录存在
            final_output_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(final_output_path), enhanced_color_img)  # 保存彩色增强图像
            print(f"图像 '{Path(image_path).name}' 已增强并保存到: {final_output_path}")

        return enhanced_color_img  # 返回彩色增强图像

    except Exception as e:
        print(f"处理图像 '{image_path}' 时发生错误: {e}")
        return None


def batch_apply_clahe(input_dir, output_dir, clip_limit=2.0, tile_grid_size=(8, 8)):
    """
    批量对目录中的所有图片应用CLAHE算法。

    Args:
        input_dir (str): 包含原始图像的目录路径。
        output_dir (str): 增强后图像保存的目录路径。
        clip_limit (float): 对比度限制阈值。
        tile_grid_size (tuple): 图像被分割成的小块网格大小。
    """
    Path(output_dir).mkdir(parents=True, exist_ok=True)  # 确保输出目录存在

    supported_formats = ('.png', '.jpg', '.jpeg', '.bmp', '.tiff')

    print(f"开始批量处理目录: {input_dir}")
    print(f"CLAHE参数: clip_limit={clip_limit}, tile_grid_size={tile_grid_size}")

    processed_count = 0
    skipped_count = 0

    for filename in os.listdir(input_dir):
        file_path = Path(input_dir) / filename
        if file_path.is_file() and filename.lower().endswith(supported_formats):
            output_file_path = Path(output_dir) / filename

            # 调用单张图片处理函数
            enhanced_img = apply_clahe(
                image_path=str(file_path),
                clip_limit=clip_limit,
                tile_grid_size=tile_grid_size,
                output_path=str(output_file_path)  # 直接将输出路径传给子函数处理保存
            )

            if enhanced_img is not None:
                processed_count += 1
            else:
                skipped_count += 1
        else:
            print(f"跳过非图片文件或目录: {filename}")
            skipped_count += 1

    print("\n--- CLAHE 批量处理完成 ---")
    print(f"成功处理图片数量: {processed_count}")
    print(f"跳过文件数量: {skipped_count}")


# --- 使用示例 ---
if __name__ == "__main__":
    # 示例1: 对单张图片应用CLAHE
    print("--- 单张图片CLAHE示例 ---")
    single_image_path = r"E:\yolov8\ultralytics-8.2.0\2025_for_train_2\train\images\capture_20250730_211906_236.jpg"  # 替换为您的单张图片路径
    output_single_image_path = r"E:\无人机\feile_team\ying\code\python_sdk\vision"  # 替换为输出路径

    # 请确保 'path/to/your/single_image.jpg' 文件存在，并修改其路径
    # 例如：single_image_path = 'E:/UAV/2025_for_train/train/images/capture_20250730_214740_380.jpg'

    # 尝试处理单张图片
    enhanced_img_single = apply_clahe(
        image_path=single_image_path,
        clip_limit=3.0,      # 对比度限制，可以调整
        tile_grid_size=(8, 8),  # 分块大小，可以调整
        output_path=output_single_image_path
    )
    if enhanced_img_single is not None:
        cv2.imshow("Original vs Enhanced (Single)", np.hstack([
            cv2.imread(single_image_path),  # 显示原图
            enhanced_img_single  # 显示增强后的图
        ]))
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("\n--- 批量图片CLAHE示例 ---")
    # # 示例2: 对整个目录的图片应用CLAHE
    # input_images_directory = 'path/to/your/input_images_folder'  # 替换为包含您原始图片的目录
    # output_images_directory = 'path/to/your/output_enhanced_images_folder'  # 替换为增强后图片保存的目录

    # # 再次强调：请确保路径中不含中文字符！
    # # 例如：
    # # input_images_directory = 'E:/UAV/2025_for_train/train/images'
    # # output_images_directory = 'E:/UAV/2025_for_train_clahe/train/images'

    # # 尝试批量处理图片
    # batch_apply_clahe(
    #     input_dir=input_images_directory,
    #     output_dir=output_images_directory,
    #     clip_limit=2.5,     # 批量处理可以调整不同的参数
    #     tile_grid_size=(10, 10)
    # )

    # print("\nCLAHE处理辅助完成，请检查输出目录。")
