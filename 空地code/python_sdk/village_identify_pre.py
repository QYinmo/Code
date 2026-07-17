import cv2
import numpy as np


sift = cv2.SIFT_create()


def extract_features_from_path(image_path):
    image = cv2.imread(image_path)
    # image = cv2.equalizeHist(image)
    keypoints, descriptors = sift.detectAndCompute(image, None)
    return descriptors


def extract_features(image):
    # gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    keypoints, descriptors = sift.detectAndCompute(image, None)
    return descriptors


def match_images(descriptors1, descriptors2):  # 计算两张图片的描述符之间的匹配
    bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)
    matches = bf.match(descriptors1, descriptors2)
    return matches


def get_similar_img(img, img_ref_descriptors):
    new_image_descriptors = extract_features(img)
    match_para = 0
    match_id = -1
    for i, descriptors in enumerate(img_ref_descriptors):
        matches = match_images(new_image_descriptors, descriptors)
        if len(matches) > match_para:
            match_para = len(matches)
            match_id = i
        print(
            f"Image {img} compared to Image {i}: {len(matches)} matches.")
    print(f"Image {img} matches Image {match_id} most.")
    if match_id == -1:
        logger.warning("出错了")
    else:
        return match_id

# with open('image_descriptors.txt', 'w') as f:
#     for descriptor in image_descriptors:
#         f.write(' '.join(map(str, descriptor)) + '\n')

# new_image_path = "test.png"
# new_image_descriptors = extract_features_from_path(new_image_path)
# print(image_descriptors)
