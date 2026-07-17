import cv2
import numpy as np
import onnxruntime as ort
import time

# img_width = 640
# img_height = 320

DEBUG = False

def preprocess(image, inpHeight):
    # 将图像大小调整为模型输入所需的大小
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (inpHeight, inpHeight), swapRB=True, crop=False)
    return blob

def postprocess(frame, outs, confThreshold, nmsThreshold):
    frameHeight = frame.shape[0]
    frameWidth = frame.shape[1]
    classIds = []
    confidences = []
    boxes = []
    centers = []
    x_factor = frameWidth / 320
    y_factor = frameHeight / 320

    # print(frameHeight, frameWidth)
    for detection in outs:
        # print(detection)
        score = detection[4:]
        classid = np.argmax(score)
        score = max(score)
        # if score > 0.5:
            # print(score)
            # print(classid)
        # max_score = np.amax(scores)
        # classId = detection[5]
        # score = score[0]
        # confidence = scores[classId]
        if score > confThreshold:
            # print(len(detection))
            # time.sleep(1)
            center_x = int((detection[0])*x_factor)
            center_y = int((detection[1])*y_factor)
            width = int(detection[2]*x_factor)
            height = int(detection[3]*y_factor)
            left = int(center_x - width/2)
            top = int(center_y - height/2)
            centers.append([int(center_x), int(center_y)])
            classIds.append(classid)
            confidences.append(float(score))
            boxes.append([left, top, width, height])

    indices = cv2.dnn.NMSBoxes(boxes, confidences, confThreshold, nmsThreshold)
    
    result = []
    
    for i in indices:
        # i = i[0]
        box = boxes[i]
        left, top, width, height = box[0], box[1], box[2], box[3]

        res_center_x = centers[i][0]
        res_center_y = centers[i][1]
        res_confidence = confidences[i]
        res_classsid = classIds[i]

        result.append([res_center_x, res_center_y, res_confidence, res_classsid])

        if DEBUG:
            cv2.rectangle(frame, (left, top), (left + width, top + height), (255, 178, 50), 3)

    return result



def infer_onnx(model_session, inp_image, inpHeight=320, confThreshold=0.3, nmsThreshold=0.2):
    '''
    res: [center_x, center_y, confidence, classid]
    '''
    # 加载ONNX模型
    # ort_session = ort.InferenceSession(model_path)
    ort_session = model_session

    # 读取图像
    # image = cv2.imread(image_path)
    image = inp_image
    blob = preprocess(image, inpHeight)
    # print(blob.shape)

    # 执行推理
    outputs = ort_session.run(None, {'images': blob})
    # print(len(outputs))
    # print(outputs[0].shape)
    outputs = outputs[0][0]
    # print(outputs.shape)
    outputs = outputs.transpose(1, 0)

    # 后处理
    res = postprocess(image, outputs, confThreshold, nmsThreshold)

    # 显示图像
    if DEBUG:
        cv2.imshow("Detected Objects", image)
        print(res)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    #print("inside",res)
    return  res

# 使用示例

if __name__ == '__main__':
    DEBUG = True
    model_path = 'best.onnx'
    ort_session = ort.InferenceSession(model_path,providers=['CPUExecutionProvider'])
    # image_path = 'IMG_9086.JPG'
    # infer_onnx(model_path, image_path)

    # 开启摄像头
    cap = cv2.VideoCapture(0)
    #提高曝光度

    cap.set(cv2.CAP_PROP_EXPOSURE, 0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cap.set(cv2.)

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        # frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = infer_onnx(model_session=ort_session, inp_image=frame)
        # cv2.imshow('frame', result)
        time.sleep(0.01)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
