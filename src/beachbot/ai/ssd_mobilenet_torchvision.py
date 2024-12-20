from beachbot.config import logger
from .debrisdetector import DebrisDetector
import torch

try:
    import torchvision

    import numpy as np

    import os
    from os import listdir
    from os.path import isfile, join
    import yaml

    class SSDMobileNetTorchvision(DebrisDetector):
        _description = """
        SSD implementation published by Nvidia based on Pytorch and torch Hub.\n
        """

        def __init__(self, model_file, use_accel=True) -> None:
            super().__init__(None)
            if "." in model_file.split(os.path.sep)[-1]:
                model_folder = os.path.dirname(os.path.realpath(model_file))
            else:
                model_folder = model_file
            model_type = "nvidia_ssd"
            with open(model_folder + "/export_info.yaml", "r") as stream:
                export_info = yaml.safe_load(stream)
                self.img_height = export_info["img_heigt_export"]
                self.img_width = export_info["img_width_export"]
                self.model_type = export_info.get("model_version", model_type)

            try:
                self.net = torchvision.models.detection.ssdlite320_mobilenet_v3_large(
                    weights=torchvision.models.detection.SSDLite320_MobileNet_V3_Large_Weights.DEFAULT
                )
                # self.net = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', self.model_type)
                # self.netutils = torch.hub.load('NVIDIA/DeepLearningExamples:torchhub', 'nvidia_ssd_processing_utils')
            except Exception as ex:
                print("Error:", ex)

            self.list_classes = []
            cls_list = torchvision.models.detection.SSDLite320_MobileNet_V3_Large_Weights.DEFAULT.meta[
                "categories"
            ]
            for clsnr in range(len(cls_list)):
                self.list_classes.append(cls_list[clsnr])
            self.num_classes = len(self.list_classes)

            # self.net.conf = 0.25  # NMS confidence threshold
            # self.net.iou = 0.45  # NMS IoU threshold
            # self.net.agnostic = False  # NMS class-agnostic
            # self.net.multi_label = False  # NMS multiple labels per box
            # self.net.classes = None  # (optional list) filter by class, i.e. = [0, 15, 16] for COCO persons, cats and dogs
            # self.net.max_det = 1000  # maximum number of detections per image
            # self.net.amp = True  # Automatic Mixed Precision (AMP) inference

            if use_accel and torch.cuda.is_available():
                self.net.cuda()
                # self.net.amp = True  # Automatic Mixed Precision (AMP) inference
            else:
                self.net.cpu()
            self.net.eval()

            self.dtype = np.float32

        def apply_model(self, inputs, confidence_threshold=0.2, units_percent=True):
            # self.net.conf = confidence_threshold  # NMS confidence threshold
            row, col, _ = inputs.shape
            inputs = (
                np.swapaxes(np.swapaxes(inputs, 0, -1), -2, -1)[None, :, :, :] / 255.0
            )
            if inputs.dtype != self.dtype:
                inputs = inputs.astype(self.dtype)
            with torch.no_grad():
                results = self.net([torch.tensor(inputs[0,:,:,:], device=next(self.net.parameters()).device)])



        
            result_boxes=results[0]['boxes'].cpu().numpy()
            result_scores = results[0]['scores'].cpu().numpy()
            result_labels = results[0]['labels'].cpu().numpy()


            result_class_ids = []
            result_confidences = []
            result_boxes_out=[]
            for i in range(result_boxes.shape[0]):
                if result_scores[i] > confidence_threshold:
                    result_class_ids.append(int(result_labels[i]))
                    result_confidences.append(float(result_scores[i]))
                    left = result_boxes[i, 0]
                    top = result_boxes[i, 1]
                    width = result_boxes[i, 2] - result_boxes[i, 0]
                    height = result_boxes[i, 3] - result_boxes[i, 1]
                    if units_percent:
                        # percent estimate, relative to image size
                        left /= inputs.shape[1]
                        top /= inputs.shape[0]
                        width /= inputs.shape[1]
                        height /= inputs.shape[0]
                    else:
                        # pixel coordinates, rond float estimates
                        left = round(left)
                        top = round(top)
                        width = round(width)
                        height = round(height)
                    bbox = np.array([left, top, width, height])
                    result_boxes_out.append(bbox)

            return result_class_ids, result_confidences, result_boxes_out

    DebrisDetector.add_model("SSDMobilenet_Torchvision", SSDMobileNetTorchvision)

except ModuleNotFoundError as ex:
    logger.warning(
        "torchvision not installed or not available! SSDMobileNetTorchvision not available!"
    )
