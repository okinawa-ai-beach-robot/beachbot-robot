from typing import List

from beachbot.config import logger
from beachbot.control.controllerselector import ControllerSelector
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.control.robotcontroller import BoxDef
from beachbot.robot.robotinterface import RobotInterface


class Hackathon(ControllerSelector):
    def __init__(self):
        super().__init__()

        # list of potential target objects
        potential_targets = {
            0: "person",
            1: "bicycle",
            2: "car",
            3: "motorcycle",
            4: "airplane",
            5: "bus",
            6: "train",
            7: "truck",
            8: "boat",
            9: "traffic light",
            10: "fire hydrant",
            11: "stop sign",
            12: "parking meter",
            13: "bench",
            14: "bird",
            15: "cat",
            16: "dog",
            17: "horse",
            18: "sheep",
            19: "cow",
            20: "elephant",
            21: "bear",
            22: "zebra",
            23: "giraffe",
            24: "backpack",
            25: "umbrella",
            26: "handbag",
            27: "tie",
            28: "suitcase",
            29: "frisbee",
            30: "skis",
            31: "snowboard",
            32: "sports ball",
            33: "kite",
            34: "baseball bat",
            35: "baseball glove",
            36: "skateboard",
            37: "surfboard",
            38: "tennis racket",
            39: "bottle",
            40: "wine glass",
            41: "cup",
            42: "fork",
            43: "knife",
            44: "spoon",
            45: "bowl",
            46: "banana",
            47: "apple",
            48: "sandwich",
            49: "orange",
            50: "broccoli",
            51: "carrot",
            52: "hot dog",
            53: "pizza",
            54: "donut",
            55: "cake",
            56: "chair",
            57: "couch",
            58: "potted plant",
            59: "bed",
            60: "dining table",
            61: "toilet",
            62: "tv",
            63: "laptop",
            64: "mouse",
            65: "remote",
            66: "keyboard",
            67: "cell phone",
            68: "microwave",
            69: "oven",
            70: "toaster",
            71: "sink",
            72: "refrigerator",
            73: "book",
            74: "clock",
            75: "vase",
            76: "scissors",
            77: "teddy bear",
            78: "hair drier",
            79: "toothbrush",
        }

        # Task 1: Create a list of objects to be picked up in order:
        self.target_order = [
            potential_targets[0],
            potential_targets[1],
            potential_targets[2],
        ]

        # current target is the first target:
        self.current_target = self.target_order[0]
        logger.debug("ControllerSelector Starting with target " + self.current_target)

        # update the sub-controller to approach the current target
        self.approachDebris.targetfilter = self.current_target

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        # 1.) If current control mode if approaching
        if self.controller is self.approachDebris:

            # execute controller and test if it completed it's task:
            if self.controller.update(robot, detections) == RESULT.SUCCESS:

                # select pickup controller as next controller:
                self.controller = self.pickup

        # 2.) If the current control mode if pick-up
        elif self.controller is self.pickup:

            # execute controller pick-up and test if it is still busy with picking the object
            if self.controller.update(robot, detections) != RESULT.BUSY:

                # Check if current_target no longer visible (not visible=picked up and placed in basked, if still visible it was dropped!)
                if not self.target_visible(detections):

                    self.next_target()

                # if picking up failed or succeeded, continue with approaching objects again next!
                self.controller = self.approachDebris

        return RESULT.BUSY
