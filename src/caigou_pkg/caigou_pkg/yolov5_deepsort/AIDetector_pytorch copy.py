import torch
import numpy as np
from caigou_pkg.yolov5_deepsort.models.experimental import attempt_load
from caigou_pkg.yolov5_deepsort.utils.general import non_max_suppression, scale_boxes
from caigou_pkg.yolov5_deepsort.utils.BaseDetector import baseDet
from caigou_pkg.yolov5_deepsort.utils.torch_utils import select_device
from caigou_pkg.yolov5_deepsort.utils.datasets import letterbox

class Detector(baseDet):

    def __init__(self):
        super(Detector, self).__init__()
        self.init_model()
        self.build_config()

    def init_model(self):

        self.weights = 'weights/best_yolov5.pt'
        self.device = '0' if torch.cuda.is_available() else 'cpu'
        self.device = select_device(self.device)
        # model = attempt_load(self.weights, map_location=self.device)
        model = attempt_load(self.weights, device=self.device)
        model.to(self.device).eval()
        model.half()
        # torch.save(model, 'test.pt')
        self.m = model
        self.names = model.module.names if hasattr(
            model, 'module') else model.names

    def preprocess(self, img):

        img0 = img.copy()
        img = letterbox(img, new_shape=self.img_size)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half()  # 半精度
        img /= 255.0  # 图像归一化
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        return img0, img

    def detect(self, im):

        im0, img = self.preprocess(im)

        pred = self.m(img, augment=False)[0]
        pred = pred.float()
        pred = non_max_suppression(pred, self.threshold, 0.4)

        pred_boxes = []

        # 读取 labellist.txt 文件，并将类别标签存储在列表中
        label_list = []
        with open('label_list.txt', 'r') as file:
            for line in file:
                label_list.append(line.strip())

        for det in pred:
            if det is not None and len(det):
                # print(f"det = ", det)
                det[:, :4] = scale_boxes(
                    img.shape[2:], det[:, :4], im0.shape).round()

                for *x, conf, cls_id in det:
                    # lbl = self.names[int(cls_id)]
                    lbl = label_list[int(cls_id)]  # 使用列表获取类别标签
                    # print(f"lbl = ", lbl)
                    # if not lbl in ['yellow_small_cube', 'car', 'truck']:
                    #     continue
                    x1, y1 = int(x[0]), int(x[1])
                    x2, y2 = int(x[2]), int(x[3])
                    # print(x1, y1, x2, y2, lbl, conf)
                    pred_boxes.append(
                        (x1, y1, x2, y2, lbl, conf))
        # print(f"pred_boxes = ", pred_boxes)
        return im, pred_boxes

