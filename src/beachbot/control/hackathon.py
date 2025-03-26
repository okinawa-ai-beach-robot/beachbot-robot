from beachbot.control.controllerselector import ControllerSelector
from beachbot.config import logger
from beachbot.robot.robotinterface import RobotInterface
from typing import List
from beachbot.control.robotcontroller import BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT

class Hackathon(ControllerSelector):
    def __init__(self):
        super().__init__()
        self.target_order = ["bottle", "cup", "sports ball"]
        self.current_target = self.target_order[0]
        logger.debug("ControllerSelector Starting with target " + self.current_target)
        self.approachDebris.targetfilter = self.current_target

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        if self.controller is self.approachDebris:
            if self.controller.update(robot, detections) == RESULT.SUCCESS:
                self.controller = self.pickup
        elif self.controller is self.pickup:
            if self.controller.update(robot, detections) != RESULT.BUSY:
                # Check if current_target no longer visible
                if not self.target_visible(detections):
                    # Consider this target successfully picked up
                    self.next_target()
                #self.next_target()
                self.controller = self.approachDebris
        return RESULT.BUSY

