import cv2
import numpy as np
from ultralytics import YOLO
model_path = r"D:\daima\Drone_Train\python_sdk\best.onnx"
cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_EXPOSURE, -4.5)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 450)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 250)
conf = 0.5
model = YOLO(model_path, task='detect')
while True:
    ret, im0 = cap.read()
    # id.using_openvino(model_path_xml, model_path_bin, im0)
    results = model(source=im0, show=False, conf=conf, save=False,
                    verbose=False, stream=True, device='cpu')
    processed_results = []
    if results is not None:
        for result in results:
            if result.boxes is not None and len(result.boxes.cpu().numpy()) > 0:
                boxes = result.boxes.cpu().numpy()
                # x y 归一化坐标
                normalized_xy = boxes.xywhn[:, :2]
                classes = boxes.cls
                confs = boxes.conf
                # print(normalized_xy, classes, confs)
                annotated_frame = result.plot()  # 自动绘制boxes/labels
                cv2.imshow('YOLOv8 Real-Time Detection', annotated_frame)
                for i in range(len(classes)):
                    processed_results.append([
                        normalized_xy[i][0],       # x坐标
                        normalized_xy[i][1],       # y坐标
                        int(classes[i]),            # 类别
                        float(confs[i])            # 置信度
                    ])

                # 转换为numpy数组便于处理
                processed_results = np.array(processed_results)

                # 每组保留一个
                # unique_classes = np.unique(processed_results[:, 2])
                # final_results = []

                # for cls in unique_classes:
                #     cls_results = processed_results[processed_results[:, 2] == cls]
                #     max_conf_idx = np.argmax(cls_results[:, 3])
                #     best_result = cls_results[max_conf_idx]
                #     final_results.append(best_result)

                # 总共保留一个
                cls_results = processed_results
                max_conf_idx = np.argmax(cls_results[:, 3])
                best_result = cls_results[max_conf_idx]
                final_results.append(best_result)
                final_results = [list(row) for row in final_results]
                # 转换为列表形式
                final_results = [list(row) for row in final_results]

                # 打印最终结果
                for res in final_results:
                    print(
                        f"类别: {int(res[2])}, 坐标: ({res[0]:.5f}, {res[1]:.5f}), 置信度: {res[3]:.5f}")
            else:
                cv2.imshow('YOLOv8 Real-Time Detection', im0)
                print("0")
    # if res is not None:
    #     for result in res:
    #         boxes = res.boxes.xyxy  # 获取所有边界框
    #         confs = res.boxes.conf  # 获取所有置信度
    #         cls_ids = res.boxes.cls  # 获取所有类别ID
    #         print(boxes, confs, cls_ids)
    # outs = id.process_detections(res)
    # print(outs)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
