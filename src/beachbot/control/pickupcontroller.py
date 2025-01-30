from typing import List
from ..robot.robotinterface import RobotInterface
from .robotcontroller import RobotController, BoxDef
from beachbot.config import logger


class PickupController(RobotController):
    def __init__(self):
        super().__init__()

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        print("tst")
        if self.debug:
            logger.debug("Pick-up Controller Active!")
        robot.arm.pickup()
        return False
