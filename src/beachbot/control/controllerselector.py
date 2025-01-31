from typing import List
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.approachdebris import ApproachDebris
from beachbot.control.pickupcontroller import PickupController
from beachbot.config import logger


class ControllerSelector(RobotController):
    def __init__(self):
        super().__init__()

        self.controllers: dict[str, RobotController] = {
            "approach": ApproachDebris(),
            "pickup": PickupController(),
        }
        self.controller = self.controllers["approach"]

        # Collect all properties of used controllers and add to this class
        for ctrl_name in self.controllers.keys():
            self.register_child_properties(class_instance=self.controllers[ctrl_name],class_name=ctrl_name)

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        if self.controller is self.controllers["approach"]:
            # check if approachDebris is done
            if self.controller.update(robot, detections):
                self.controller = self.controllers["pickup"]
        elif self.controller is self.controllers["pickup"]:
            # Pick-up trash
            # TODO what to do afterwards?
            self.controller.update(robot, detections)
            self.controller = None  # Stop afterwards
