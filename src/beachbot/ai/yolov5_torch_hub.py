from beachbot.config import config, logger
from .debrisdetector import DebrisDetector
from .yolov5_detector import Yolo5Detector
import torch
import numpy as np

import os
from os import listdir
from os.path import isfile, join
import yaml
import time

try:

    class Yolo5TorchHub(Yolo5Detector):
        _description = """
        Official YOLOv5 implementation based on Pytorch and torch Hub.\n
        This Module can load the official as well as self-trained models.
        """

        def __init__(self, model_file=None, use_accel=True) -> None:
            super().__init__(None)
            logger.debug(f"model_file: {model_file}")

            if model_file is None:
                model_file = str(config.BEACHBOT_MODELS) + "/Original_YOLOv5s/"
            if "." in model_file:
                model_folder = os.path.dirname(os.path.realpath(model_file))
            else:
                model_folder = os.path.realpath(model_file)
            model_type = None
            with open(model_folder + "/export_info.yaml", "r") as stream:
                export_info = yaml.safe_load(stream)
                self.img_height = export_info["img_heigt_export"]
                self.img_width = export_info["img_width_export"]
                self.model_type = export_info.get("model_version", model_type)

            if self.model_type is None:
                self.num_classes = str(export_info.get("nc", 6))
                self.list_classes = export_info.get(
                    "names",
                    [
                        "other_avoid",
                        "other_avoid_boundaries",
                        "other_avoid_ocean",
                        "others_traverable",
                        "trash_easy",
                        "trash_hard",
                    ],
                )
                try:
                    if os.path.isfile(model_folder + os.path.sep + "best.pt"):
                        logger.info("Loading pt model file")
                        self.net = torch.hub.load(
                            "ultralytics/yolov5:master",
                            "custom",
                            path=model_folder + os.path.sep + "best.pt",
                        )
                    else:
                        logger.info("Loading onnx model file")
                        self.net = torch.hub.load(
                            "ultralytics/yolov5:master",
                            "custom",
                            path=model_folder + os.path.sep + "best.onnx",
                        )
                except Exception as ex:
                    logger.error(str(ex))
            else:
                logger.info(f"Loading original yolov5 model {self.model_type}")
                self.net = torch.hub.load("ultralytics/yolov5:master", self.model_type)
                self.list_classes = []
                for clsnr in range(max(list(self.net.names.keys())) + 1):
                    self.list_classes.append(self.net.names[clsnr])
                self.num_classes = len(self.list_classes)

            self.net.conf = self.conf_threshold  # NMS confidence threshold
            self.net.iou = 0.45  # NMS IoU threshold
            self.net.agnostic = False  # NMS class-agnostic
            self.net.multi_label = False  # NMS multiple labels per box
            self.net.classes = None  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
            self.net.max_det = 1000  # maximum number of detections per image
            # self.net.amp = True  # Automatic Mixed Precision (AMP) inference

            if use_accel and torch.cuda.is_available():
                logger.info("Inference on CUDA is available")
                self.net.cuda()
                self.net.amp = True  # Automatic Mixed Precision (AMP) inference
            else:
                logger.info("Inference on CPU")
                self.net.cpu()

            self.dtype = np.float32

            self.downscale = 4
            self.register_property("downscale",max_value=4, min_value=1, descr="Perform downscaling of original image input resolution before processing")
            logger.debug(f"model_file: {model_file}")

        def apply_model(self, inputs, units_percent=True):
            self.net.conf = self.conf_threshold  # NMS confidence threshold
            row, col, _ = inputs.shape
            downscaler=max(min(self.downscale, 4), 1)


            if self.debug:
                logger.debug(f"Postprocessing of image: downscale resolution by factor {downscaler}.")

            with torch.no_grad():
                start = time.perf_counter()
                results = self.net([inputs[..., ::-1]], size=320) # detect on BGR->RGB pixel format
                end = time.perf_counter()
                logger.debug(f"Inference time: {end - start:.6f}")

            res = results.xyxy[0].numpy(force=True)
            result_class_ids = []
            result_confidences = []
            result_boxes = []


            for i in range(res.shape[0]):
                result_class_ids.append(int(res[i, 5]))
                result_confidences.append(res[i, 4])
                left = res[i, 0]
                top = res[i, 1]
                width = res[i, 2] - res[i, 0]
                height = res[i, 3] - res[i, 1]
                if units_percent:
                    # percent estimate, relative to image (input!) size
                    left /= inputs.shape[1]
                    top /= inputs.shape[0]
                    width /= inputs.shape[1]
                    height /= inputs.shape[0]
                else:
                    # pixel coordinates, round float estimates
                    left = round(left)
                    top = round(top)
                    width = round(width)
                    height = round(height)
                bbox = np.array([left, top, width, height])
                result_boxes.append(bbox)

                if self.debug:
                    logger.debug(f"Detection with threshold {self.net.conf}, on {inputs.shape} image, boxes are {result_boxes}, confidences are {result_confidences}")

            return result_class_ids, result_confidences, result_boxes

    DebrisDetector.add_model("YOLOv5_Torch_Hub", Yolo5TorchHub)
    DebrisDetector.add_model("YOLOv5", Yolo5TorchHub)



    class BeachbotYolo5TorchHub(Yolo5TorchHub):
        """Default beachbot model
        default model file set to: beachbot_yolov5s_beach-cleaning-object-detection__v8-yolotrain__yolov5pytorch_640_finetune
        uses cuda acceleration if available
        """
        def __init__(self, model_file=None, use_accel=True):
            if model_file is None:
                model_file = str(config.BEACHBOT_MODELS) + "/beachbot_yolov5s_beach-cleaning-object-detection__v8-yolotrain__yolov5pytorch_640_finetune/"
            super().__init__(model_file, use_accel)

except ModuleNotFoundError as ex:
    logger.error(
        "Torch hub yolo5 model not installed or not available! Yolo5TorchHub not available!"
    )
