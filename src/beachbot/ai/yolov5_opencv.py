from .debrisdetector import DebrisDetector
from .yolov5_detector import Yolo5Detector
import cv2
import numpy as np

import os
from os import listdir
from os.path import isfile, join
import yaml

class Yolo5OpenCV(Yolo5Detector):
    _description="""
    YOLOv5 implementation based on OpenCV framework.\n
    Supports hardware acceleration via CUDA if available on platform.
    """

    def __init__(self, model_file, use_accel=True) -> None:
        super().__init__(model_file)
        if not model_file.endswith(".onnx"):
            model_file += "/best.onnx"
        model_folder = os.path.dirname(os.path.realpath(model_file))
        with open(model_folder + "/export_info.yaml", 'r') as stream:
            export_info = yaml.safe_load(stream)
            img_height = export_info['img_heigt_export']
            img_width = export_info['img_width_export']
            num_classes = str(export_info.get('nc', 6))
            list_classes = export_info.get('names',["other_avoid","other_avoid_boundaries","other_avoid_ocean","others_traverable","trash_easy","trash_hard"])
        print("Exported ONNX model operates on images of size ", img_width, "x", img_height, "[wxh] pixels")
        print("Dataset defines", num_classes, "classes ->\n", list_classes)

        model_cfg_file="?"
        for file in os.listdir(model_folder):
            if file.endswith(".yaml"):
                model_cfg_file = model_folder + str(file)
                break

        self.net = cv2.dnn.readNet(model_file, model_cfg_file)
        if use_accel:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA_FP16)
        else:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

        self.img_width = img_width
        self.img_height = img_height
        # if "float16" in input_type:
        #     self.dtype=np.float16
        # elif "float32" in input_type:
        #     self.dtype=np.float32


    def apply_model(self, inputs, confidence_threshold=0.2, units_percent=True):  
        img = self._crop_and_scale_image(inputs)
        row, col, _ = img.shape
        _max = max(col, row)
        result = np.zeros((_max, _max, 3), np.uint8)
        result[0:row, 0:col] = img
        scale = 1.0/255.0 # convert byte color 0-255 to float value range 0-1
        blob = cv2.dnn.blobFromImage(result, scalefactor=scale, size=(self.img_width,self.img_height), mean=(0,0,0), swapRB=False, crop=False) #swapRB: RGB<->BGR conversion!
        self.net.setInput(blob)

        prediction = self.net.forward()
        #image was made into square image, here, rescale the box positions and sizes to fit input dimension?
        for rown in range(prediction.shape[1]):
            rx = prediction[0][rown][0]
            ry = prediction[0][rown][1]
            rw = prediction[0][rown][2]
            rh = prediction[0][rown][3]

            rxn = (_max/col)*rx
            ryn = (_max/row)*ry
            rwn = (_max/col)*rw
            rhn = (_max/row)*rh

            prediction[0][rown][0] = rxn
            prediction[0][rown][1] = ryn
            prediction[0][rown][2] = rwn
            prediction[0][rown][3] = rhn

        result_class_ids, result_confidences, result_boxes = self._wrap_detection(prediction[0], confidence_threshold=confidence_threshold)
        self._map_resuts_to_input_image(result_boxes, inputs, units_percent=units_percent)
        return result_class_ids, result_confidences, result_boxes
    

DebrisDetector.add_model("YOLOv5", Yolo5OpenCV)
