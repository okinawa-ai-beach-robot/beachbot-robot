from typing import List
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.approachdebris import ApproachDebris
from beachbot.control.pickupcontroller import PickupController
from beachbot.control.adaptivepicker import AdaptivePickupController
from beachbot.config import logger


class ControllerSelector(RobotController):
    def __init__(self, approachDebris: ApproachDebris = None, pickup: PickupController = None):
        super().__init__()
        self.target_order = []

        self.do_adative_picking = False
        self.register_property(
            "do_adative_picking",
            descr="If true, use AdaptivePickupController(), otherwise use PickupController()",
        )

        if approachDebris is None:
            self.approachDebris = ApproachDebris(self)
        if pickup is None:
            self.pickup = PickupController(self)
        self.controllers: dict[str, RobotController] = {
            "approach": self.approachDebris,
            "pickup": self.pickup,
        }
        self.controller = self.controllers["approach"]

        # Collect all properties of used controllers and add to this class
        for ctrl_name in self.controllers.keys():
            self.register_child_properties(
                class_instance=self.controllers[ctrl_name], class_name=ctrl_name
            )

    def property_changed_callback(self, name):
        super().property_changed_callback(name)

        if name == "do_adative_picking":
            # re-instanciate class for pickup.
            if self.do_adative_picking:
                logger.info("Activate adative picker")
                self.controllers["pickup"] = AdaptivePickupController(self)
            else:
                logger.info("Activate default picker")
                self.controllers["pickup"] = PickupController(self)

            # Collect all properties of used controllers and add to this class
            for ctrl_name in self.controllers.keys():
                self.register_child_properties(
                    class_instance=self.controllers[ctrl_name], class_name=ctrl_name
                )

    def next_target(self, robot: RobotInterface):
        self.target_order.pop(0)
        if len(self.target_order) > 0:
            self.current_target = self.target_order[0]
            self.approachDebris.targetfilter = self.current_target
            if self.debug:
                logger.info(f"Next target object {self.current_target} selected!")
        else:
            logger.info("All targets picked up!")
            self.current_target = None
            self.approachDebris.targetfilter = None
            self.controller = None
            robot.set_target_velocity(0, 0)

    def target_visible(self, detections: List[BoxDef] = None) -> bool:
        if detections is None:
            return False
        for det in detections:
            if det.class_name == self.current_target:
                return True
        return False
