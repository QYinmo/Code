import glob
import os
import cv2
import numpy as np
import onnxruntime as ort
 
 
classes = {
    0: 'farm',
    1:'lake',
    2:'mudslide',
    3:'hillfire',
    4:'mountain',
    5:'village',
    6:'river'

}
class Colors:
    """Ultralytics color palette https://ultralytics.com/."""
 
    def __init__(self):
        """Initialize colors as hex = matplotlib.colors.TABLEAU_COLORS.values()."""
        hexs = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
                '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb(f'#{c}') for c in hexs]
        # print(self.palette)
        self.n = len(self.palette)
 
    def __call__(self, i, bgr=False):
        """Converts hex color codes to rgb values."""
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c
 
    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))
 
 
colors = Colors()  # create instance for 'from utils.plots import colors'
 
 
def letterbox(
        im,
        new_shape,
        color=(114, 114, 114),
        auto=False,
        scaleFill=False,
        scaleup=True,
        stride=32,
):
    """
    Resize and pad image while meeting stride-multiple constraints
    Returns:
        im (array): (height, width, 3)
        ratio (array): [w_ratio, h_ratio]
        (dw, dh) (array): [w_padding h_padding]
    """
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):  # [h_rect, w_rect]
        new_shape = (new_shape, new_shape)
 
    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)
 
    # Compute padding
    ratio = r, r  # wh ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))  # w h
    dw, dh = (
        new_shape[1] - new_unpad[0],
        new_shape[0] - new_unpad[1],
    )  # wh padding
 
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])  # [w h]
        ratio = (
            new_shape[1] / shape[1],
            new_shape[0] / shape[0],
        )  # [w_ratio, h_ratio]
 
    dw /= 2  # divide padding into 2 sides
    dh /= 2
    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(
        im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )
    return im, ratio, (dw, dh)
 
 
def rescale_coords(boxes, image_shape, input_shape):
    image_height, image_width = image_shape
    input_height, input_width = input_shape
 
    scale = min(input_width / image_width, input_height / image_height)
 
    pad_w = (input_width - image_width * scale) / 2
    pad_h = (input_height - image_height * scale) / 2
 
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - pad_w) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - pad_h) / scale
 
    boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, image_width)
    boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, image_height)
 
    return boxes.astype(int)
 
 
def preprocess(image, input_shape):
    # Resize
    input_img = letterbox(image, input_shape)[0]
    # Transpose
    input_img = input_img[..., ::-1].transpose(2, 0, 1)
    # Expand
    input_img = input_img[np.newaxis, :, :, :].astype(np.float32)
    # Contiguous
    input_img = np.ascontiguousarray(input_img)
    # Norm
    blob = input_img / 255.0
    return blob
 
 
def postprocess(outs, conf_thres, image_shape, input_shape):
    # Filtered by conf
    outs = outs[outs[:, 4] >= conf_thres]
 
    # Extract
    boxes = outs[:, :4]
    scores = outs[:, -2]
    labels = outs[:, -1].astype(int)
 
    # Rescale
    boxes = rescale_coords(boxes, image_shape, input_shape)
 
    return boxes, scores, labels

def infer_and_draw(ori_img, session , LABEL_NAMES, thresh=0.65):
    input_shape = (320, 320)
    image_shape = ori_img.shape[:2]
    blob = preprocess(ori_img, input_shape)

    # Inference
    outs = session.run(None, {'images': blob})[0][0]

    # Postprocess
    boxes, scores, labels = postprocess(outs, thresh, image_shape, input_shape)
    print("boxes",boxes,"scores",scores,"labels",labels)

    # 保存结果
    for label, score, box in zip(labels, scores, boxes):
        label_text = f'{classes[label]}: {score:.2f}'
        color = colors(label,True)
        cv2.rectangle(ori_img, (box[0], box[1]), (box[2], box[3]), color, thickness=2, lineType=cv2.LINE_AA)
        cv2.putText(ori_img, label_text, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    
    return boxes, labels
 
def main():
    conf_thres = 0.35
    # input_shape = (320, 320)
    save_path = 'C:\\Users\\29943\\Downloads\\infer\\infer_Rst'
    os.makedirs(save_path,exist_ok=True)
    model_path = r"./detect.onnx"

    ort_model = ort.InferenceSession(model_path)

    cap = cv2.VideoCapture(0)  # 0 是摄像头的设备索引，如果你有多个摄像头，可能需要改变这个值
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640.0)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480.0)
    #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
    cap.set(cv2.CAP_PROP_EXPOSURE,650)

    while True:
        ret, img = cap.read()  # 从摄像头读取一帧
        if not ret:
            continue

        img_center_x = int(img.shape[1]/2)
        img_center_y = int(img.shape[0]/2)-40
        boxes, labels = infer_and_draw(img, ort_model, 0.2)
        for label, box in zip(labels, boxes):
            center_x = int((box[0] + box[2]) / 2)
            center_y = int((box[1] + box[3]) / 2)
            if int((abs(center_x - img_center_x)**2+abs(center_y - img_center_y)**2)**(1/2)) < 80:
                #logger.info(f"label:{label}")
                print(center_x, center_y)
                print(label)
                

        # # Preprocess
        # image_shape = im0.shape[:2]
        # blob = preprocess(im0, input_shape)

        # # Inference
        # outs = ort_model.run(None, {'images': blob})[0][0]

        # # Postprocess
        # boxes, scores, labels = postprocess(outs, conf_thres, image_shape, input_shape)

        # # 保存结果
        # for label, score, box in zip(labels, scores, boxes):
        #     label_text = f'{classes[label]}: {score:.2f}'
        #     color = colors(label,True)
        #     cv2.rectangle(im0, (box[0], box[1]), (box[2], box[3]), color, thickness=2, lineType=cv2.LINE_AA)
        #     cv2.putText(im0, label_text, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        cv2.imshow('image', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # 如果按下 'q' 键，退出循环
            break

    cap.release()  # 释放摄像头资源
    cv2.destroyAllWindows()  # 关闭所有 OpenCV 窗口
# def main():
#     conf_thres = 0.25
#     input_shape = (320, 320)
#     image_path = 'C:\\Users\\29943\\Downloads\\infer\\infer'
#     save_path = image_path + '_Rst'
#     os.makedirs(save_path,exist_ok=True)
#     model_path = r"C:\\Users\\29943\\Downloads\\infer\\infer\\best.onnx"
 
#     ort_model = ort.InferenceSession(model_path)
#     imgs = glob.glob(os.path.join(image_path,'*.JPG'))
#     imgs.sort()
#     for img in imgs:
#         imgname = img.split('\\')[-1]
#         # Preprocess
#         im0 = cv2.imread(img)
#         image_shape = im0.shape[:2]
#         blob = preprocess(im0, input_shape)
 
#         # Inference
#         outs = ort_model.run(None, {'images': blob})[0][0]
 
#         # Postprocess
#         boxes, scores, labels = postprocess(outs, conf_thres, image_shape, input_shape)
 
#         # 保存结果
#         for label, score, box in zip(labels, scores, boxes):
#             label_text = f'{classes[label]}: {score:.2f}'
#             color = colors(label,True)
#             cv2.rectangle(im0, (box[0], box[1]), (box[2], box[3]), color, thickness=2, lineType=cv2.LINE_AA)
#             cv2.putText(im0, label_text, (box[0], box[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
 
#         # cv2.imshow('image', im0)
#         # cv2.waitKey(0)
#         cv2.imwrite(save_path+'\\'+imgname, im0)
 
 
if __name__ == '__main__':
    main()