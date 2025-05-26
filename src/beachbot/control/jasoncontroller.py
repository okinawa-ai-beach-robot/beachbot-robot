from typing import List
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.approachdebris import ApproachDebris
from beachbot.control.pickupcontroller import PickupController
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.config import logger
from beachbot.control.roamaround import RoamAround


class JasonController(RobotController):
    def __init__(self):
        super().__init__()

        self.controllers: dict[str, RobotController] = {
            "approach": ApproachDebris(),
            "pickup": PickupController(),
            "roam" : RoamAround(),
        }
        self.controller = self.controllers["roam"]

        # Collect all properties of used controllers and add to this class
        for ctrl_name in self.controllers.keys():
            self.register_child_properties(class_instance=self.controllers[ctrl_name],class_name=ctrl_name)

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        if self.controller is self.controllers["roam"]:
            logger.info("Roam")
            if self.controller.update(robot, detections) == RESULT.SUCCESS:
                self.controller = self.controllers["approach"]
        elif self.controller is self.controllers["approach"]:
            logger.info("Approach")
            if self.controller.update(robot, detections) == RESULT.SUCCESS:
                self.controller = self.controllers["pickup"]
            else:
                self.controller = self.controllers["roam"]
        elif self.controller is self.controllers["pickup"]:
            logger.info("Pickup")
            if self.controller.update(robot, detections) == RESULT.SUCCESS:
                self.controller = self.controllers["roam"]




