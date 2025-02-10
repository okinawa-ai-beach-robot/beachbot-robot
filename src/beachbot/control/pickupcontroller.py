from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef 
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.config import logger


class PickupController(RobotController):
    def __init__(self):
        super().__init__()

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        logger.info("pickup")
        robot.arm.pickup()
        logger.info("tossing")
        robot.arm.toss()
        robot.arm.go_home()

        #TODO assess situation and adjust return value appropriately, was pickup successful?
        return RESULT.SUCCESS
