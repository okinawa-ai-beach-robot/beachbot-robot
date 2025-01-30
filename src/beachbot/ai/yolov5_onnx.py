from beachbot.config import logger
from .debrisdetector import DebrisDetector
from .yolov5_detector import Yolo5Detector
import onnxruntime
import numpy as np


class Yolo5Onnx(Yolo5Detector):
    _description = """
    YOLOv5 implementation based on ONNXruntime framework.\n
    Supports hardware acceleration via CUDA if available on platform.\n
    Tensorrt acceleration is not yet implemented.
    """

    def __init__(self, model_file, use_accel=True) -> None:
        super().__init__(model_file)
        providers = ["CPUExecutionProvider"]
        if (
            use_accel
            and "TensorrtExecutionProvider" in onnxruntime.get_available_providers()
        ):
            logger.info("TODO: Ignore Tensorrt for now!!")
            providers = ["CUDAExecutionProvider"]
        elif (
            use_accel
            and "CUDAExecutionProvider" in onnxruntime.get_available_providers()
        ):
            providers = ["CUDAExecutionProvider"]
        else:
            logger.info("No Gpu acceleration availabe!")
        logger.info("DL providers are:" + str(providers))
        if not model_file.endswith(".onnx"):
            model_file += "/best.onnx"
        self.session = onnxruntime.InferenceSession(model_file, providers=providers)
        if self.session is None:
            raise ValueError("Failed to load the model " + model_file)
        input_shapes = self.session.get_inputs()[0].shape
        input_type = self.session.get_inputs()[0].type
        self.img_width = input_shapes[3]
        self.img_height = input_shapes[2]
        logger.info("model type is " + str(input_type))
        if "float16" in input_type:
            self.dtype = np.float16
        elif "float32" in input_type or "float" in input_type:
            self.dtype = np.float32

    def apply_model(self, inputs, units_percent=True):
        img = self._crop_and_scale_image(inputs)
        img = np.swapaxes(np.swapaxes(img, 0, -1), -2, -1)[None, :, :, :] / 255.0
        if img.dtype != self.dtype:
            img = img.astype(self.dtype)
        prediction = self.session.run(None, {"images": img})

        result_class_ids, result_confidences, result_boxes = self._wrap_detection(
            prediction[0][0], confidence_threshold=self.conf_threshold
        )
        self._map_resuts_to_input_image(
            result_boxes, inputs, units_percent=units_percent
        )
        return result_class_ids, result_confidences, result_boxes


DebrisDetector.add_model("YOLOv5", Yolo5Onnx)
