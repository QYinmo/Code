import village_identify_pre
import cv2
import numpy as np
import onnx_detect
import onnxruntime as ort


class_name = {
    0: 'vocano',
    1: 'mountain',
    2: 'lake',
    3: 'river',
    4: 'farmland',
    5: 'village',
    6: 'mudslide'

}


model_path = "./best.onnx"

conf_thres = 0.8


def start_camera_task(self):
    threading.Thread(target=self.camera_task, daemon=True).start()


def identify_area(self, img, model):
    try:
        results = model(source=img, show=False, conf=conf_thres, save=False,
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

                    # 打印最终结果
                    for res in final_results:
                        print(
                            f"类别: {int(res[2])}, 坐标: ({res[0]:.5f}, {res[1]:.5f}), 置信度: {res[3]:.5f}")
                        return res[2]
                else:
                    print("没东西")
                    return None
    except:
        logger.warning("死了")
        return None


def camera_task(self):
    while True:
        ret, img = self.cam.read()
        # img = img[50:200, 100:300]
        if self.identify_status:
            res_pre = -1
            res = self.identify_area(img)
            # logger.info(f"这是{res}")
            if res == None:
                continue
            else:
                if res == res_pre:
                    self.area_count += 1
                    self.area_timeout = 0
                else:
                    self.area_timeout += 1
                if self.area_count > 3:
                    self.area_count = 0
                    self.area_timeout = 0
                    self.identify_status = False
                    if self.started:
                        self.area_res = int(res)
                        print("这是", class_name[int(res)])
                        self.next_point_event.set()
                elif self.area_timeout > 8:
                    self.area_timeout = 0
                    self.area_count = 0
                    self.identify_status = False
                    if self.started:
                        self.area_res = -1
                        logger.warning("太难了，我不会")
                        self.next_point_event.set()
