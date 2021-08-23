import cv2
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.data import MetadataCatalog, build_detection_test_loader
import glob
import os
import yaml
import numpy as np
import time
from pycocotools import mask
from detectron2.projects import point_rend
from py_filter_detections import filter_detections

class DetectronInference:
    """
    This class instantiates detectron. As long as it exists, each call to detect() will be done on the same instance, thus
    reducingin initilization time.
    """

    def __init__(self, inputs_dir_path):
        self.base_path = os.getcwd()
        self.inputs_dir_path = inputs_dir_path

        # load config file path
        config_list = glob.glob(os.path.join(inputs_dir_path + "/*.yaml"))

        class_names = [yaml_file for yaml_file in config_list if 'class_names' in yaml_file]
        with open(class_names[0], 'r')as f:
            parsed_class_names = yaml.load(f, Loader=yaml.FullLoader)
            num_classes = len(parsed_class_names['MODEL']['NAMES'])
            self.num_classes = num_classes
            self.labels_list = parsed_class_names['MODEL']['NAMES']

        config_list = [yaml_file for yaml_file in config_list if 'class_names' not in yaml_file]
        assert len(
            config_list) == 1, 'ERROR: Exactly one .yaml file has to be present in detectron2_configs directory, not counting class_names.yaml'
        self.config_file_path = config_list[0]

        # load weights path
        weights_list = glob.glob(os.path.join(inputs_dir_path + "/*.pth"))
        assert len(
            weights_list) == 1, 'ERROR: Exactly one .pth weights file has to be present in detectron2_inference_weights ' \
                                'directory '
        self.weights_path = weights_list[0]

        # set config and weights
        self.config_file = get_cfg()
        point_rend.add_pointrend_config(self.config_file)
        self.config_file.MODEL.ROI_HEADS.NUM_CLASSES = num_classes
        self.config_file.MODEL.POINT_HEAD.NUM_CLASSES = num_classes

        self.config_file.merge_from_file(self.config_file_path)
        self.config_file.MODEL.WEIGHTS = self.weights_path
        self.config_file.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.7
        self.predictor = DefaultPredictor(self.config_file)
        assert len(self.labels_list) == int(self.config_file.get('MODEL').get('ROI_HEADS').get('NUM_CLASSES')), \
            "This model requires " + str(self.config_file.get('MODEL').get('ROI_HEADS').get('NUM_CLASSES')) + \
            " class labels but " + str(len(self.labels_list)) + " were provided"

    def detect_image(self, image):
        """
        This method does inference on a provided image using Detectron2.
        :param image: input image in opencv format
        :return: dict containing classes, boxes, masks and scores for all detections in order
        """
        inf_start = time.time()
        print(image.shape)
        print(image)
        predictions = self.predictor(image)

        inf_end = time.time()
        print("Detectron object inference: " + str(inf_end - inf_start))

        res_start = time.time()
        outputs = predictions['instances'].get_fields()
        masks = outputs['pred_masks'].cpu().numpy().astype(np.uint8)
        encoded = list()
        k = 0
        for x in masks:
            # k+=1
            # x *= 255
            # x = np.fliplr(x)
            # x = np.rot90(x)
            # res = mask.encode(np.asfortranarray(x))
            # res['counts'] = res['counts'].decode()
            # encoded.append(res)
            # n = 'img'
            # cv2.imwrite('imgpy')
            encoded.append(x)
        MetadataCatalog.get(self.config_file.DATASETS.TRAIN[0]).thing_classes = self.labels_list

        outputs = {
            "classes": [self.labels_list[class_digit] for class_digit in
                        outputs['pred_classes'].cpu().numpy().tolist()],
            # "bboxes": outputs['pred_boxes'].tensor.cpu().numpy().tolist(),
            "masks": encoded
            # "scores": outputs['scores'].cpu().numpy().tolist(),
            # "class_ids": self.labels_list,
        }
        # DO FILTER DETECTIONS HERE
        filtered_masks = filter_detections(outputs)

        res_end = time.time()
        print("Detectron object gpu result retrieval: ", str(res_end - res_start))
        return filtered_masks

    def get_label_list(self):
        with open(os.path.join(self.inputs_dir_path, "class_names.yaml"), 'r') as stream:
            try:
                labels_list = yaml.safe_load(stream)['MODEL']['NAMES']
                return labels_list
            except yaml.YAMLError as exc:
                print(exc)

