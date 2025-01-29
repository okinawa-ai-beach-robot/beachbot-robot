from typing import List

from ..robot.robotinterface import RobotInterface
from .robotcontroller import RobotController, BoxDef
from ..utils.controllercollection import PIDController


class PickupController(RobotController):
    def __init__(self):
        super().__init__()
        

        example_default_float_var = 0.0


        self._register_property("varnameExample", example_default_float_var)


    def _property_changed_callback(self, name):
        if name=="varnameExample":
            print("var changed!!", self.get_property(name))
        else:
            super()._property_changed_callback(name)

    def update(self, robot: RobotInterface, detections: List[BoxDef]=None, debug=False):
          print("TODO pick up movement")
        
        