from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.config import logger
from concurrent.futures import ThreadPoolExecutor


class PickupController(RobotController):
    def __init__(self):
        super().__init__()
        self.executor = ThreadPoolExecutor(max_workers=1)

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        logger.info("pickup")
        self.executor.submit(robot.arm.pickup)
        logger.info("tossing")
        self.executor.submit(robot.arm.toss)
        self.executor.submit(robot.arm.go_home)
        return True
