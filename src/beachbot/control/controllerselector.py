from typing import List, Union
from .robotcontroller import RobotController, BoxDef
from ..robot.robotinterface import RobotInterface
from ..utils.controllercollection import PIDController
from .approachdebris import ApproachDebris
from .pickupcontroller import PickupController

class ControllerSelector(RobotController):
    def __init__(self):
        super().__init__()

        self.controllers : dict[str,RobotController] = {
            "approach": ApproachDebris(),
            "pickup": PickupController(),
            #....
            # add here list of used sub-controller modules ... 
        }
        self.controller = self.controllers["approach"]

        # Collect all properties of used controllers and add to this class
        for ctrl_name in self.controllers.keys():
            self.register_child_properties(class_instance=self.controllers[ctrl_name],class_name=ctrl_name)









    def update(self, robot: RobotInterface, detections: List[BoxDef]=None):
        if self.controller is self.controllers["approach"]:
            # check if approachDebris is done
            if self.controller.update(robot, detections):
                self.controller = self.controllers["pickup"]
        elif self.controller is self.controllers["pickup"]:
            # Pick-up trash
            # TODO what to do afterwards?
            self.controller.update(robot, detections)
            self.controller=None # Stop afterwards


            
