from typing import List
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.approachdebris import ApproachDebris
from beachbot.control.pickupcontroller import PickupController
from beachbot.control.adaptivepicker import AdaptivePickupController
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.config import logger


class ControllerSelector(RobotController):
    def __init__(self):
        super().__init__()

        self.do_adative_picking=False
        self.register_property("do_adative_picking", descr="If true, use AdaptivePickupController(), otherwise use PickupController()")

        self.controllers: dict[str, RobotController] = {
            "approach": ApproachDebris(),
            "pickup": PickupController(),
        }
        self.controller = self.controllers["approach"]

        # Collect all properties of used controllers and add to this class
        for ctrl_name in self.controllers.keys():
            self.register_child_properties(class_instance=self.controllers[ctrl_name],class_name=ctrl_name)


    def property_changed_callback(self, name):
        super().property_changed_callback(name)

        if name=="do_adative_picking":
            # re-instanciate class for pickup.
            if self.do_adative_picking:
                logger.info("Activate adative picker")
                self.controllers["pickup"]=AdaptivePickupController()
            else:
                logger.info("Activate default picker")
                self.controllers["pickup"]=PickupController()

            # Collect all properties of used controllers and add to this class
            for ctrl_name in self.controllers.keys():
                self.register_child_properties(class_instance=self.controllers[ctrl_name],class_name=ctrl_name)

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        if self.controller is self.controllers["approach"]:
            # check if approachDebris is done
            if self.controller.update(robot, detections) == RESULT.SUCCESS:
                logger.info("approachDebris done, switching to pickup...")
                self.controller = self.controllers["pickup"]
        elif self.controller is self.controllers["pickup"]:
            if self.controller.update(robot, detections) != RESULT.BUSY:
                logger.info("pickup done, switching to approachDebris...")
                self.controller = self.controllers["approach"]

        return RESULT.BUSY

