import sys
import time
from pathlib import Path
import numpy as np
import cv2
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())  # add yolov5/ to path

from .utils.general import check_img_size, non_max_suppression, scale_coords
from .utils.plots import colors, plot_one_box
from .utils.torch_utils import time_sync


class Ensemble(torch.nn.ModuleList):
    # Ensemble of models
    def __init__(self):
        super().__init__()

    def forward(self, x, augment=False, profile=False, visualize=False):
        y = []
        for module in self:
            y.append(module(x, augment, profile, visualize)[0])
        y = torch.cat(y, 1)  # nms ensemble
        return y, None  # inference, train output


class YoloInference():
    @torch.no_grad()
    def load_model(self, map_location=None, inplace=True):
        from models.yolo import Detect, Model

        # Loads an ensemble of models weights=[a,b,c] or a single model weights=[a] or weights=a
        model = Ensemble()
        model_path = str(FILE).split("/")
        model_path[-1] = 'yolov5s.pt'
        model_path = '/'.join(model_path)
        model_path = Path(str(model_path).strip().replace("'", ''))
        ckpt = torch.load(model_path, map_location=map_location)  # load
        model.append(ckpt['ema' if ckpt.get('ema') else 'model'].float().fuse().eval())  # FP32 model

        # Compatibility updates
        for m in model.modules():
            if type(m) in [torch.nn.Hardswish, torch.nn.LeakyReLU, torch.nn.ReLU, torch.nn.ReLU6, torch.nn.SiLU, Detect,
                           Model]:
                m.inplace = inplace  # pytorch 1.7.0 compatibility
        return model[-1]

    @torch.no_grad()
    def __init__(self, imgsz=640,  # inference size (pixels)
                 conf_thres=0.25,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 visualize=True,  # show results
                 classes=0,  # filter by class: --class 0, or --class 0 2 3
                 agnostic_nms=False,  # class-agnostic NMS
                 augment=False,  # augmented inference
                 hide_labels=False,  # hide labels
                 hide_conf=False,  # hide confidences
                 half=True,  # use FP16 half-precision inference
                 show_inf_time=False
                 ):
        ###################################
        ########### INIT BLOCK ############
        ###################################
        self.half = half
        self.show_inf_time = show_inf_time
        self.hide_labels = hide_labels
        self.hide_conf = hide_conf
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.classes = classes
        self.visualize = visualize
        self.agnostic_nms = agnostic_nms
        self.augment = augment
        self.device = torch.device('cuda:0')
        self.model = self.load_model(map_location=self.device)  # load FP32 model

        # Load model
        self.stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(imgsz, s=self.stride)  # check image size
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
        if self.half:
            self.model.half()  # to FP16

        cudnn.benchmark = True  # set True to speed up constant image size inference
        self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
        ###################################
        ######### END INIT BLOCK ##########
        ###################################

    @torch.no_grad()
    def inference(self, img):
        t0 = time.time()
        if self.visualize:
            im0 = img.copy()
        im_shape = img.shape
        img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_sync()
        pred = self.model(img,
                          augment=self.augment,
                          visualize=False)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=100)
        t2 = time_sync()

        # Process detections
        boxes = list()
        for i, det in enumerate(pred):  # detections per image

            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im_shape).round()
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    boxes.append(list(map(lambda x: int(x), xyxy)))
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (
                        self.names[c] if self.hide_conf else f'{self.names[c]} {conf:.2f}')
                    if self.visualize:
                        plot_one_box(xyxy, im0, label=label, color=colors(c, True), line_thickness=2)
                print(boxes)

        if self.visualize:
            cv2.imshow('Stream', im0)
            cv2.waitKey(1)  # 1 millisecond
        if self.show_inf_time:
            print(f'Done. ({time.time() - t0:.3f}s)')

        return boxes


if __name__ == "__main__":
    y = YoloInference()
    img = cv2.imread('/home/avena/PycharmProjects/yolo5/detect_input.png')
    for i in range(100):
        y.inference(img)
