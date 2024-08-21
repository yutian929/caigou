import numpy as np
import cv2
from openvino.runtime import Core
from caigou_pkg.yolov5_deepsort.utils.general import non_max_suppression, scale_boxes
from caigou_pkg.yolov5_deepsort.utils.BaseDetector import baseDet
from caigou_pkg.yolov5_deepsort.utils.datasets import letterbox

class Detector(baseDet):
    def __init__(self):
        super(Detector, self).__init__()
        self.init_model()
        self.build_config()

    def init_model(self):
        # OpenVINO 相关初始化
        # caigou_ws/src/caigou_pkg/caigou_pkg/yolov5_deepsort/weights/track_model/track_20240615.xml
        self.model_path = '/home/caigou/workspace/caigou_ws/src/caigou_pkg/caigou_pkg/yolov5_deepsort/weights/best_openvino_model_0705n/best.xml'  # 替换为 OpenVINO IR 格式模型路径
        self.CLASSES = {0: 'box'}
        self.core = Core()
        self.device = "GPU"  # 或 "AUTO" 选择设备
        self.ov_config = {"GPU_DISABLE_WINOGRAD_CONVOLUTION": "YES"}
        self.net = self.core.compile_model(self.model_path, device_name=self.device, config=self.ov_config)
        self.output_node = self.net.outputs[0]

    def preprocess(self, img):
        img0 = img.copy()
        [height, width, _] = img0.shape
        length = max(height, width)
        image = np.zeros((length, length, 3), np.uint8)
        image[0:height, 0:width] = img0
        self.scale = length / 640

        # 创建Blob
        blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255, size=(640, 640), swapRB=True)

        return img0, blob
    

    def detect(self, im):
        # DEBUG
        # im = cv2.imread('/home/caigou/workspace/hs_project/YOLOv8-Openvino-master/demo.jpg')
        
        im0, blob = self.preprocess(im)
        # print(f"blob = {blob}")

        # 执行推理
        outputs = self.net.create_infer_request().infer({self.net.inputs[0]: blob})[self.output_node]
        # print(f"outputs = {outputs}")
        outputs = np.array([cv2.transpose(outputs[0])])
        # print(f"outputs = {outputs}")

        # OpenVINO 输出处理
        rows = outputs.shape[1]

        boxes = []
        scores = []
        class_ids = []

        for i in range(rows):
            classes_scores = outputs[0][i][4:]
            (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
            if maxScore >= 0.25:
                box = [
                    outputs[0][i][0] - (0.5 * outputs[0][i][2]), 
                    outputs[0][i][1] - (0.5 * outputs[0][i][3]),
                    outputs[0][i][2], 
                    outputs[0][i][3]
                ]
                boxes.append(box)
                scores.append(maxScore)
                class_ids.append(maxClassIndex)

        result_boxes = cv2.dnn.NMSBoxes(boxes, scores, 0.25, 0.45, 0.5)
        # print(f"result_boxes = {result_boxes}")

        pred_boxes = []
        # for i in range(rows):
        #     classes_scores = outputs[0][i][4:]
        #     (minScore, maxScore, minClassLoc, (x, maxClassIndex)) = cv2.minMaxLoc(classes_scores)
        #     if maxScore >= 0.25:  # 阈值过滤
        #         box = [
        #             outputs[0][i][0] - (0.5 * outputs[0][i][2]), 
        #             outputs[0][i][1] - (0.5 * outputs[0][i][3]),
        #             outputs[0][i][2], 
        #             outputs[0][i][3]
        #         ]
        #         x1, y1 = int(box[0] * im0.shape[1]), int(box[1] * im0.shape[0])
        #         x2, y2 = int((box[0] + box[2]) * im0.shape[1]), int((box[1] + box[3]) * im0.shape[0])
        #         lbl = label_list[maxClassIndex]
        #         pred_boxes.append((x1, y1, x2, y2, lbl, maxScore))

        for i in range(len(result_boxes)):
            index = result_boxes[i]
            box = boxes[index]
            detection = {
                'class_id': class_ids[index],
                'class_name': self.CLASSES[class_ids[index]],
                'confidence': scores[index],
                'box': box,
                'scale': self.scale
            }
            pred_boxes.append((round(box[0] * self.scale), round(box[1] * self.scale),
                      round((box[0] + box[2]) * self.scale), round((box[1] + box[3]) * self.scale),
                      self.CLASSES[class_ids[index]]))
           
 
        return pred_boxes
    
    # if __name__ == "__main__":
    #     det = Detector()
    #     img = cv2.imread('/home/caigou/workspace/hs_project/YOLOv8-Openvino-master/demo.jpg')
    #     img, pred_boxes = det.detect(img)
    #     print(pred_boxes)
    #     # cv2.imshow('result', img)
    #     cv2.imwrite('result.jpg', img)
    #     # cv2.waitKey(0)
    #     # cv2.destroyAllWindows()

