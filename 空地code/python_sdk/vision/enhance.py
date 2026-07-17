import os
import cv2
import numpy as np
import imgaug.augmenters as iaa
from imgaug.augmentables.bbs import BoundingBox, BoundingBoxesOnImage
from pathlib import Path

# --- 辅助函数（与之前相同，无需修改） ---


def load_yolo_annotation(label_path, img_width, img_height):
    """加载YOLO格式的标注文件，并转换为imgaug的BoundingBox对象列表。"""
    bbs = []
    if not os.path.exists(label_path):
        # 如果对应的标签文件不存在，返回空的BoundingBoxesOnImage对象
        return BoundingBoxesOnImage([], shape=(img_height, img_width))

    with open(label_path, 'r') as f:
        for line in f.readlines():
            parts = list(map(float, line.strip().split()))
            class_id = int(parts[0])
            x_center, y_center, width, height = parts[1:]

            # 将归一化坐标转换为像素坐标
            x1 = int((x_center - width / 2) * img_width)
            y1 = int((y_center - height / 2) * img_height)
            x2 = int((x_center + width / 2) * img_width)
            y2 = int((y_center + height / 2) * img_height)

            # imgaug要求x1, y1 <= x2, y2
            x1, x2 = min(x1, x2), max(x1, x2)
            y1, y2 = min(y1, y2), max(y1, y2)

            bbs.append(BoundingBox(x1=x1, y1=y1, x2=x2, y2=y2, label=class_id))

    return BoundingBoxesOnImage(bbs, shape=(img_height, img_width))


def save_yolo_annotation(output_label_path, bbs_aug, img_width, img_height):
    """将imgaug的BoundingBox对象列表转换回YOLO格式并保存。"""
    with open(output_label_path, 'w') as f:
        for bb in bbs_aug.bounding_boxes:
            # 确保边界框在图像范围内
            # imgaug在某些几何变换后可能会导致BBox部分或完全超出图像
            # 可以选择在这里过滤掉太小的或完全超出图像的bbox

            # 检查边界框的宽度和高度是否大于某个阈值。
            # 较新版本的imgaug可能没有is_valid属性。
            # 并且我们通常会过滤掉太小的或无意义的边界框。
            # 可以根据需要调整min_bbox_size_pixels
            min_bbox_size_pixels = 1  # 定义最小边界框像素尺寸，小于此尺寸的框将被丢弃

            if bb.width > min_bbox_size_pixels and bb.height > min_bbox_size_pixels:  # 确保bbox有最小尺寸
                # 转换回归一化坐标
                x_center = (bb.x1 + bb.x2) / 2.0 / img_width
                y_center = (bb.y1 + bb.y2) / 2.0 / img_height
                width = (bb.x2 - bb.x1) / img_width
                height = (bb.y2 - bb.y1) / img_height

                # 限制坐标在 0-1 之间，防止越界，并处理可能因浮点数导致的小于0或大于1的情况
                x_center = np.clip(x_center, 0.0, 1.0)
                y_center = np.clip(y_center, 0.0, 1.0)
                width = np.clip(width, 0.0, 1.0)
                height = np.clip(height, 0.0, 1.0)

                # 重新检查归一化后的宽高是否仍然有效（避免裁剪后变成负值或0）
                if width > 0 and height > 0:
                    f.write(
                        f"{bb.label} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")


def augment_subset(input_base_dir, output_base_dir, subset_name, num_aug_per_image, aug_sequence):
    """
    对一个子集（train或val）进行数据增强。
    :param input_base_dir: 原始数据集的根目录 (例如 'path/to/your/dataset/2025_for_train')
    :param output_base_dir: 增强后数据集的根目录 (例如 'path/to/your/augmented_dataset/2025_for_train_aug')
    :param subset_name: 子集名称 ('train' 或 'val')
    :param num_aug_per_image: 每张原始图片生成多少张增强图片
    :param aug_sequence: imgaug的增强序列
    """
    input_image_dir = Path(input_base_dir) / subset_name / 'images'
    input_label_dir = Path(input_base_dir) / subset_name / 'labels'

    output_image_dir = Path(output_base_dir) / subset_name / 'images'
    output_label_dir = Path(output_base_dir) / subset_name / 'labels'

    # 创建输出目录（如果不存在）
    output_image_dir.mkdir(parents=True, exist_ok=True)
    output_label_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n--- 开始处理 {subset_name} 子集 ---")

    image_files = [f for f in os.listdir(input_image_dir) if f.lower().endswith(
        ('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]

    print(f"在 '{input_image_dir}' 中找到 {len(image_files)} 张图片。")

    for i, img_filename in enumerate(image_files):
        img_path = input_image_dir / img_filename
        label_filename = Path(img_filename).stem + ".txt"
        label_path = input_label_dir / label_filename

        image = cv2.imread(str(img_path))  # cv2.imread需要字符串路径
        if image is None:
            print(f"警告: 无法读取图片 {img_path}，跳过。")
            continue
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # imgaug通常使用RGB

        img_height, img_width, _ = image.shape
        bbs = load_yolo_annotation(str(label_path), img_width, img_height)

        for j in range(num_aug_per_image):
            # 对图像和标注同时进行增强
            # augment_bounding_boxes=False 表示不随机选择边界框进行增强，而是对所有框都应用
            image_aug, bbs_aug = aug_sequence(image=image, bounding_boxes=bbs)

            # 保存增强后的图像
            output_img_filename = f"{Path(img_filename).stem}_aug{j}{Path(img_filename).suffix}"
            output_img_path = output_image_dir / output_img_filename
            cv2.imwrite(str(output_img_path), cv2.cvtColor(
                image_aug, cv2.COLOR_RGB2BGR))

            # 保存增强后的标注
            output_label_filename = f"{Path(img_filename).stem}_aug{j}.txt"
            output_label_path = output_label_dir / output_label_filename
            save_yolo_annotation(str(output_label_path),
                                 bbs_aug, image_aug.shape[1], image_aug.shape[0])

        if (i + 1) % 50 == 0 or (i + 1) == len(image_files):  # 每处理50张或处理完时打印进度
            print(f"  已处理 {i + 1}/{len(image_files)} 张图片。")

    print(f"--- {subset_name} 子集数据增强完成！---")


# --- 使用示例 ---
if __name__ == "__main__":
    # 原始数据集的**根目录**
    # 假设您的原始数据在 'path/to/your/dataset/2025_for_train/train/images' 等
    # 请根据您的实际路径修改
    ORIGINAL_DATASET_BASE = r"E://无人机//2025_train_changjiao1"

    # 增强后数据集的**输出根目录**
    # 增强后的数据将保存到 'path/to/your/augmented_dataset/2025_for_train_aug/train/images' 等
    AUGMENTED_DATASET_BASE = r"E://无人机//2025_train_changjiao+enhance"

    # 定义数据增强序列
    # 你可以根据需要调整这些增强器的种类、概率和强度
    augmentation_sequence = iaa.Sequential([
        iaa.Fliplr(0.5),  # 水平翻转50%的图像
        iaa.Sometimes(0.5, iaa.Flipud(0.2)),  # 50%的概率垂直翻转20%的图像
        iaa.Sometimes(0.7, iaa.Rot90((1, 3))),  # 70%的概率随机旋转90, 180, 270度
        iaa.Sometimes(0.5, iaa.Affine(
            scale={"x": (0.8, 1.2), "y": (0.8, 1.2)},  # 缩放图像80%-120%
            translate_percent={
                "x": (-0.1, 0.1), "y": (-0.1, 0.1)},  # 平移图像-10%到+10%
            rotate=(-15, 15),  # 旋转-15到15度
            shear=(-8, 8),  # 剪切-8到8度
            cval=(0, 255)  # 填充颜色范围
        )),
        iaa.Sometimes(0.5, iaa.GaussianBlur(sigma=(0, 1.0))),  # 50%的概率高斯模糊
        iaa.Sometimes(0.5, iaa.LinearContrast((0.75, 1.5))),  # 50%的概率调整对比度
        iaa.Sometimes(0.5, iaa.AdditiveGaussianNoise(
            loc=0, scale=(0.0, 0.05*255), per_channel=0.5)),  # 50%的概率添加高斯噪声
        # 50%的概率调整亮度 (乘以一个因子)
        iaa.Sometimes(0.5, iaa.Multiply((0.8, 1.2), per_channel=0.2)),
        iaa.Sometimes(0.5, iaa.WithBrightnessChannels(
            iaa.Add((-30, 30)))),  # 50%的概率调整亮度 (加减值)
        # iaa.Sometimes(0.5, iaa.CoarseDropout(0.02, size_percent=0.1, per_channel=0.5)), # 50%的概率随机去除部分区域（谨慎使用）
    ], random_order=True)  # 随机应用增强器的顺序

    # --- 对训练集进行增强 ---
    # 每张训练图片生成3个增强样本
    augment_subset(
        input_base_dir=ORIGINAL_DATASET_BASE,
        output_base_dir=AUGMENTED_DATASET_BASE,
        subset_name='train',
        num_aug_per_image=3,  # 为训练集生成更多增强样本
        aug_sequence=augmentation_sequence
    )

    # --- 对验证集进行增强（可选，通常只进行少量或不进行增强）---
    # 验证集通常不进行大规模增强，以保持其与真实世界数据的相似性，用于评估模型的泛化能力
    # 如果验证集图片太少，可以进行少量增强（例如每张1个样本）
    augment_subset(
        input_base_dir=ORIGINAL_DATASET_BASE,
        output_base_dir=AUGMENTED_DATASET_BASE,
        subset_name='val',
        num_aug_per_image=1,  # 为验证集生成1个增强样本（即复制并可能应用简单增强）
        aug_sequence=augmentation_sequence  # 可以使用相同的增强序列，或定义一个更简单的
    )

    print("\n所有子集的数据增强处理完成！")
    print(f"增强后的数据集保存在: {AUGMENTED_DATASET_BASE}")
