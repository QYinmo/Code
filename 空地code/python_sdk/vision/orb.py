import cv2
import numpy as np

# 读取图片
img1 = cv2.imread('vocano.png', cv2.IMREAD_GRAYSCALE)
img2 = cv2.imread('test.png', cv2.IMREAD_GRAYSCALE)

# 2. 创建ORB检测器
orb = cv2.ORB_create()

# 3. 检测关键点和计算描述符
kp1, des1 = orb.detectAndCompute(img1, None)
kp2, des2 = orb.detectAndCompute(img2, None)

# 4. 检查描述符是否为None
if des1 is None or des2 is None:
    print("Error: One of the descriptors is None, feature extraction failed.")
else:
    # 5. 确保描述符的数据类型一致
    des1 = np.uint8(des1)
    des2 = np.uint8(des2)

    # 6. 使用暴力匹配器（Brute-Force Matcher）
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    # 7. 匹配描述符
    matches = bf.match(des1, des2)
    print("匹配度", len(matches))
    # 8. 根据匹配的距离排序，距离越小，匹配越好
    matches = sorted(matches, key=lambda x: x.distance)

    # 9. 只保留前10个匹配
    good_matches = matches[:10]

    # 10. 绘制匹配结果
    img_matches = cv2.drawMatches(
        img1, kp1, img2, kp2, good_matches, None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # 11. 显示结果
    cv2.imshow("ORB Matches", img_matches)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
