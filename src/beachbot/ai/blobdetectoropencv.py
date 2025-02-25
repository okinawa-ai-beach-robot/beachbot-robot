from .debrisdetector import DebrisDetector
from .yolov5_detector import Yolo5Detector
import cv2
import numpy as np


class BlobDetectorOpenCV(DebrisDetector):
    _description="""
    BlobDetectorOpenCV implementation of simple blob detector.
    """

    def __init__(self, model_file=None, use_accel=True, minArea = 1000, maxArea = 200000) -> None:
        super().__init__(None)
        params = cv2.SimpleBlobDetector_Params()
        params.filterByCircularity = False
        params.filterByConvexity = False
        params.filterByInertia = False

        params.filterByArea = True
        params.minArea = minArea
        params.maxArea = maxArea
        params.filterByColor = False

        # Opencv uses 0-179 for hue value 0-359
        def deg2byte(val):
            return round(180*(val/360))
        self.lower_blue = np.array([deg2byte(190), 60, 60])
        self.upper_blue = np.array([deg2byte(255), 255, 255]) 
        self.lower_green = np.array([deg2byte(92), 60, 60])
        self.upper_green = np.array([deg2byte(145), 255, 255]) 
        self.lower_yellow = np.array([deg2byte(45), 60, 60])
        self.upper_yellow = np.array([deg2byte(71), 255, 255]) 
        self.lower_pink = np.array([deg2byte(270), 60, 60])
        self.upper_pink = np.array([deg2byte(340), 255, 255]) 
        #params.minThreshold = 10
        #params.thresholdStep = 1
        # params.blobColor = 255
        self.blobclasses = [
            (self.lower_blue, self.upper_blue, "blue_blob"),
            (self.lower_green, self.upper_green, "green_blob"),
            (self.lower_yellow, self.upper_yellow, "yellow_blob"),
            (self.lower_pink, self.upper_pink, "pink_blob"),
        ]

        self.params=params
        self.detector = cv2.SimpleBlobDetector_create(params)

        self.num_classes = 6
        self.list_classes = [
                        "green_blob",
                        "yellow_blob",
                        "pink_blob",
                        "blue_blob"
                        ]
    






    def apply_model(self, inputs, units_percent=True, debug=False):  
        img = inputs
        row, col, _ = img.shape
        s_img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)  
        if debug:
            cv2.imwrite("img.png", s_img)

        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        if debug:
            cv2.imwrite("hsv.png", hsv_img)

        result_boxes = []
        result_class_ids=[]
        result_confidences=[]


        for lower,upper,clsname in self.blobclasses:
            mask_blob = cv2.inRange(hsv_img, lower, upper)
            mask_blob = cv2.erode(mask_blob, None, iterations=0)
            mask_blob = cv2.dilate(mask_blob, None, iterations=0)
            s_img = cv2.bitwise_and(s_img,s_img,mask = mask_blob)
            if debug:
                cv2.imwrite(f"mask_{clsname}.png", mask_blob)
                cv2.imwrite(f"s_img_{clsname}.png", s_img)
            keyp = self.detector.detect(mask_blob)


            for p in keyp:
                if debug:
                    print("Blob detector output:", p.pt, p.size, row, col)

                width = (p.size)/col
                height = (p.size)/row
                left = (p.pt[0]-p.size/2)/col
                top = (p.pt[1]-p.size/2)/row
                if debug:
                    print("Box from blob is:", left, top, width, height)
                bbox = np.array([left, top, width, height])
                result_boxes.append(bbox)
                result_class_ids.append(self.list_classes.index(clsname))
                result_confidences.append(1.0)


        
        return result_class_ids, result_confidences, result_boxes
    

DebrisDetector.add_model("BlobDetector", BlobDetectorOpenCV)
