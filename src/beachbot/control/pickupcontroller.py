from typing import List
from ..robot.robotinterface import RobotInterface
from .robotcontroller import RobotController, BoxDef


class PickupController(RobotController):
    def __init__(self):
        super().__init__()

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None, debug=False):
