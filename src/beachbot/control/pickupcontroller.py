from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.config import logger


class PickupController(RobotController):
    def __init__(self):
        super().__init__()

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        logger.info("pickup")
        robot.arm.pickup()
        logger.info("tossing")
        robot.arm.toss()
        return True
