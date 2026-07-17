from ultralytics import YOLO
# 加载训练好的模型，改为自己的路径
model = YOLO('./best_2.pt')
# 修改为自己的图像或者文件夹的路径
source = '20250427_150217_22.png'  # 修改为自己的图片路径及文件名
# 运行推理，并附加参数
model.predict(source, save=True)
