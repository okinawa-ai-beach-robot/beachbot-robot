import os
import yaml


from beachbot.config import config, logger


class DebrisDetector:
    _model_lib = {}
    _description = """
    Abstract base class of coastal debris classificator.\n
    can not be used direclty, please use implementation found in sublcasses.
    """

    def __init__(self, model_file=None) -> None:
        self.img_height = -1
        self.img_width = -1
        self.dtype = None
        self.num_classes = 0
        self.list_classes = []

        if model_file is not None:
            if "." in model_file:
                model_folder = os.path.dirname(os.path.realpath(model_file))
            else:
                model_folder = os.path.realpath(model_file)
            with open(model_folder + "/export_info.yaml", "r") as stream:
                export_info = yaml.safe_load(stream)
                self.img_height = export_info["img_heigt_export"]
                self.img_width = export_info["img_width_export"]
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
            logger.info(
                "Exported ONNX model operates on images of size "
                + str(self.img_width)
                + "x"
                + str(self.img_height)
                + " [wxh] pixels"
            )
            logger.info(
                "Dataset defines "
                + str(self.num_classes)
                + " classes -> "
                + str(self.list_classes)
            )

    def apply_model(self, inputs, confidence_threshold=0.2):
        raise NotImplementedError

    @staticmethod
    def draw_boxes(class_ids, confidences, boxes, image, box_config):
        colors = [
            (255, 255, 0),
            (0, 255, 0),
            (0, 255, 255),
            (255, 0, 0),
            (255, 0, 0),
            (255, 0, 0),
            (255, 0, 0),
            (255, 0, 0),
            (255, 0, 0),
            (255, 0, 0),
        ]
        for classid, confidence, box in zip(class_ids, confidences, boxes):
            if confidence >= box_config:
                color = colors[int(classid) % len(colors)]
                cv.rectangle(image, box, color, 2)
        img2 = image[:, :, ::-1]
        return img2

    @staticmethod
    def list_models_by_type(type):
        return DebrisDetector._model_lib.get(type, [])

    @staticmethod
    def add_model(type, modelcls):
        curr_list = DebrisDetector._model_lib.get(type, [])
        if modelcls not in curr_list:
            curr_list.append(modelcls)
            DebrisDetector._model_lib[type] = curr_list
            logger.info("Added class " + str(modelcls) + " with type " + str(type))

    @staticmethod
    def list_model_types():
        return list(DebrisDetector._model_lib.keys())

    @staticmethod
    def list_model_paths():
        base_path = config.BEACHBOT_MODELS
        modelfolders = [f.path for f in os.scandir(base_path) if f.is_dir()]
        # sort by date
        modelfolders.sort(
            key=lambda x: -os.path.getmtime(x),
        )
        modelfolders.insert(0, "BlobDetector")
        return modelfolders

    @staticmethod
    def get_model_type(modelpath):
        model_type = None
        if modelpath=="BlobDetector":
            return "BlobDetector"
        if modelpath is not None:
            if modelpath.endswith(os.path.sep + "best.onnx"):
                modelpath = modelpath[: -len(os.path.sep + "best.onnx")]
            # load files
            with open(modelpath + os.path.sep + "export_info.yaml", "r") as stream:
                export_info = yaml.safe_load(stream)
                model_type = export_info.get(
                    "model_type", "YOLOv5"
                )  # for backward-compatibility
        return model_type

    @staticmethod
    def list_models_by_path(modelpath):
        model_type = DebrisDetector.get_model_type(modelpath)
        return DebrisDetector.list_models_by_type(model_type)
