import cv2
import numpy as np

# 1. 读取彩色图像
img1 = cv2.imread('mountain_2.png')
img2 = cv2.imread('mountain_2_2.png')

# 2. 创建 SIFT 检测器
sift = cv2.SIFT_create()

# 3. 检测关键点和计算描述符
kp1, des1 = sift.detectAndCompute(img1, None)
kp2, des2 = sift.detectAndCompute(img2, None)

# 4. 使用暴力匹配器（Brute-Force Matcher）来匹配描述符
bf = cv2.BFMatcher(cv2.NORM_L2, crossCheck=True)

# 5. 匹配描述符
matches = bf.match(des1, des2)
print("匹配度", len(matches))
# 6. 根据匹配的距离排序，距离越小，匹配越好
matches = sorted(matches, key=lambda x: x.distance)

# 7. 只保留前10个匹配
good_matches = matches[:10]

# 8. 绘制匹配结果
img_matches = cv2.drawMatches(img1, kp1, img2, kp2, good_matches,
                              None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# 9. 显示结果
cv2.imshow("SIFT Matches", img_matches)
cv2.waitKey(0)
cv2.destroyAllWindows()
