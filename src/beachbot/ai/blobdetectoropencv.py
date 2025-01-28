from .debrisdetector import DebrisDetector
from .yolov5_detector import Yolo5Detector
import cv2
import numpy as np


class BlobDetectorOpenCV(DebrisDetector):
    _description="""
    BlobDetectorOpenCV implementation of simple blob detector.
    """

    def __init__(self, model_file=None, use_accel=True, minArea = 1000, maxArea = 200000) -> None:
        params = cv2.SimpleBlobDetector_Params()
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False

        params.filterByArea = True
        params.minArea = minArea
        params.maxArea = maxArea
        params.filterByColor = False

        self.lower_blue = np.array([60, 60, 60])
        self.upper_blue = np.array([180, 255, 255]) 
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
    






    def apply_model(self, inputs, confidence_threshold=0.2, units_percent=True, debug=False):  
        img = inputs
        row, col, _ = img.shape
        s_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  
        if debug:
            cv2.imwrite("img.png", s_img)

        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        if debug:
            cv2.imwrite("hsv.png", hsv_img)



        mask_blue = cv2.inRange(hsv_img, self.lower_blue, self.upper_blue)
        mask_blue = cv2.erode(mask_blue, None, iterations=0)
        mask_blue = cv2.dilate(mask_blue, None, iterations=0)
        s_img = cv2.bitwise_and(s_img,s_img,mask = mask_blue)
        if debug:
            cv2.imwrite("mask.png", mask_blue)
            cv2.imwrite("s_img.png", s_img)
        keyp = self.detector.detect(mask_blue)

        result_boxes = []
        result_class_ids=[]
        result_confidences=[]
        for p in keyp:
            if debug or True:
                print("Blob detector output:", p.pt, p.size, row, col)

            width = (p.size)/col
            height = (p.size)/row
            left = (p.pt[0]-p.size/2)/col
            top = (p.pt[1]-p.size/2)/row
            if debug:
                print("Box from blob is:", left, top, width, height)
            bbox = np.array([left, top, width, height])
            result_boxes.append(bbox)
            result_class_ids.append(self.list_classes.index("trash_easy"))
            result_confidences.append(1.0)


        
        return result_class_ids, result_confidences, result_boxes
    

DebrisDetector.add_model("BlobDetector", BlobDetectorOpenCV)
