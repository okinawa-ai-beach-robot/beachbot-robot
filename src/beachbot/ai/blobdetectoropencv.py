from .debrisdetector import DerbrisDetector
from .yolov5_detector import Yolo5Detector
import cv2
import numpy as np

import os
from os import listdir
from os.path import isfile, join
import yaml

class BlobDetectorOpenCV(DerbrisDetector):
    _description="""
    BlobDetectorOpenCV implementation of simple blob detector.
    """

    def __init__(self, model_file=None, use_accel=True) -> None:
        params = cv2.SimpleBlobDetector_Params()
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False

        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 100000
        params.filterByColor = False
        #params.minThreshold = 10
        #params.thresholdStep = 1
        # params.blobColor = 255
        self.params=params
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.num_classes = 6
        self.list_classes = [
                        "other_avoid",
                        "other_avoid_boundaries",
                        "other_avoid_ocean",
                        "others_traverable",
                        "trash_easy",
                        "trash_hard",]






    def apply_model(self, inputs, confidence_threshold=0.2, units_percent=True):  
        img = inputs
        row, col, _ = img.shape
        _max = max(col, row)
        s_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  
        cv2.imwrite("img.png", s_img)

        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        print(hsv_img)

        cv2.imwrite("hsv.png", hsv_img)

        lower_blue = np.array([60, 60, 60])
        upper_blue = np.array([180, 255, 255]) 

        mask_blue = cv2.inRange(hsv_img, lower_blue, upper_blue)
        mask_blue = cv2.erode(mask_blue, None, iterations=0)
        mask_blue = cv2.dilate(mask_blue, None, iterations=0)
        s_img = cv2.bitwise_and(s_img,s_img,mask = mask_blue)
        cv2.imwrite("mask.png", mask_blue)
        cv2.imwrite("s_img.png", s_img)
        keyp = self.detector.detect(mask_blue)

        result_boxes = []
        result_class_ids=[]
        result_confidences=[]
        for p in keyp:
            print(p.pt, p.size, row, col)
            left = p.pt[0]/col
            top = (row-p.pt[1])/row
            width = (p.size*2)/col
            height = (p.size*2)/row
            bbox = np.array([left, top, width, height])
            result_boxes.append(bbox)
            result_class_ids.append(self.list_classes.index("trash_easy"))
            result_confidences.append(1.0)


        
        return result_class_ids, result_confidences, result_boxes
    

DerbrisDetector.add_model("BlobDetector", BlobDetectorOpenCV)
