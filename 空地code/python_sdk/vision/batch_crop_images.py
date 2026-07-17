import os
from PIL import Image
from pathlib import Path


def batch_crop_images_by_percentage(input_dir, output_dir,
                                    crop_left_percent, crop_top_percent,
                                    crop_width_percent, crop_height_percent):
    """
    批量截取指定目录下的图片。裁剪区域通过百分比指定。

    Args:
        input_dir (str): 包含原始图片的目录路径。
        output_dir (str): 裁剪后图片保存的目录路径。
        crop_left_percent (float): 裁剪区域左侧起点相对于图片宽度的百分比 (0.0 - 1.0)。
        crop_top_percent (float): 裁剪区域顶部起点相对于图片高度的百分比 (0.0 - 1.0)。
        crop_width_percent (float): 裁剪区域宽度相对于图片宽度的百分比 (0.0 - 1.0)。
        crop_height_percent (float): 裁剪区域高度相对于图片高度的百分比 (0.0 - 1.0)。
    """
    # 确保输出目录存在，如果不存在则创建
    Path(output_dir).mkdir(parents=True, exist_ok=True)

    # 定义支持的图片格式
    supported_formats = ('.png', '.jpg', '.jpeg', '.bmp', '.gif', '.tiff')

    print(f"正在扫描目录: {input_dir}")
    print(f"裁剪区域设置 (百分比):")
    print(f"  左侧起点: {crop_left_percent*100:.2f}%")
    print(f"  顶部起点: {crop_top_percent*100:.2f}%")
    print(f"  裁剪宽度: {crop_width_percent*100:.2f}%")
    print(f"  裁剪高度: {crop_height_percent*100:.2f}%")

    processed_count = 0
    skipped_count = 0

    # 遍历输入目录中的所有文件
    for filename in os.listdir(input_dir):
        # 构建完整的文件路径
        file_path = os.path.join(input_dir, filename)

        # 检查是否是文件且是支持的图片格式
        if os.path.isfile(file_path) and filename.lower().endswith(supported_formats):
            try:
                # 打开图片
                with Image.open(file_path) as img:
                    img_width, img_height = img.size

                    # 将百分比转换为实际像素值
                    # 左上角X坐标
                    crop_x = int(img_width * crop_left_percent)
                    # 左上角Y坐标
                    crop_y = int(img_height * crop_top_percent)
                    # 裁剪宽度
                    crop_width = int(img_width * crop_width_percent)
                    # 裁剪高度
                    crop_height = int(img_height * crop_height_percent)

                    # 检查裁剪区域是否超出图片范围
                    if crop_x < 0 or crop_y < 0 or \
                       crop_x + crop_width > img_width or \
                       crop_y + crop_height > img_height:
                        print(
                            f"警告: 图片 '{filename}' (尺寸: {img_width}x{img_height}) 的裁剪区域超出其范围，跳过此图片。")
                        skipped_count += 1
                        continue

                    # 定义裁剪框：(left, upper, right, lower)
                    # right = x + width, lower = y + height
                    crop_box = (crop_x, crop_y, crop_x +
                                crop_width, crop_y + crop_height)

                    # 执行裁剪
                    cropped_img = img.crop(crop_box)

                    # 构建输出文件路径
                    output_file_path = os.path.join(output_dir, filename)

                    # 保存裁剪后的图片
                    cropped_img.save(output_file_path)
                    processed_count += 1
                    print(
                        f"已裁剪并保存: {filename} (裁剪像素: ({crop_x},{crop_y}) 到 ({crop_x+crop_width},{crop_y+crop_height}))")

            except Exception as e:
                print(f"错误: 处理图片 '{filename}' 时发生错误: {e}")
                skipped_count += 1
        else:
            # 可能是目录或其他非图片文件
            # print(f"跳过非图片文件或目录: {filename}") # 可以注释掉这行减少输出
            skipped_count += 1

    print("\n--- 裁剪完成 ---")
    print(f"成功裁剪图片数量: {processed_count}")
    print(f"跳过文件数量: {skipped_count}")


# --- 使用示例 ---
if __name__ == "__main__":
    # 配置参数
    # 1. 原始图片所在目录
    INPUT_IMAGE_DIRECTORY = "E:\无人机\pic\pic"

    # 2. 裁剪后图片保存的目录
    OUTPUT_IMAGE_DIRECTORY = "E:\无人机\pic_2"

    # 3. 裁剪区域的左上角起点百分比 (0.0 到 1.0)
    # 例如：0.1 表示从左/上 10% 的位置开始
    CROP_START_LEFT_PERCENT = 0.20  # 从图片左侧开始裁剪10%
    CROP_START_TOP_PERCENT = 0.20   # 从图片顶部开始裁剪5%

    # 4. 裁剪区域的宽度和高度百分比 (0.0 到 1.0)
    # 例如：0.8 表示裁剪图片总宽度的80%
    CROP_WIDTH_PERCENT = 0.60   # 裁剪图片总宽度的80%
    CROP_HEIGHT_PERCENT = 0.60  # 裁剪图片总高度的90%

    # 调用函数执行批量裁剪
    batch_crop_images_by_percentage(
        input_dir=INPUT_IMAGE_DIRECTORY,
        output_dir=OUTPUT_IMAGE_DIRECTORY,
        crop_left_percent=CROP_START_LEFT_PERCENT,
        crop_top_percent=CROP_START_TOP_PERCENT,
        crop_width_percent=CROP_WIDTH_PERCENT,
        crop_height_percent=CROP_HEIGHT_PERCENT
    )
