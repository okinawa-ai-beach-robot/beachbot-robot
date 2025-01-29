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

        # Collect all properties of used controllers and add to this class
        for ctrl_name in self.controllers.keys():
            ctrlcls = self.controllers[ctrl_name]
            for ctrl_name_property in ctrlcls.list_property_names():
                prop_default_val = ctrlcls.get_property(ctrl_name_property)
                prop_bounds = ctrlcls.get_property_bounds(ctrl_name_property)
                prop_min = None
                prop_max = None
                if prop_bounds is not None:
                    prop_min, prop_max = prop_bounds
                self._register_property(ctrl_name + "." + ctrl_name_property, prop_default_val, max_value=prop_max, min_value=prop_min)



    def set_property(self, name:str, value:Union[bool, float, str]):
        if "." in name:
            # forward property change to children ....
            ctrlname, propname=  name.split(".", 1)
            if ctrlname in self.controllers:
                self.controllers[ctrlname].set_property(propname, value)
            else:
                raise ValueError(f"Controller module {ctrlname} not known, can not set property {propname}")
        else:
            super().set_property(name, value)

    def get_property(self,name:str) -> Union[bool, float, str, None]:
        if "." in name:
            # retrieve property values from children ....
            ctrlname, propname=  name.split(".", 1)
            if ctrlname in self.controllers:
                return self.controllers[ctrlname].get_property(propname)
            else:
                raise ValueError(f"Controller module {ctrlname} not known, can not get property {propname}")
        else:
            return super().get_property(name)





    def update(self, robot: RobotInterface, detections: List[BoxDef]=None, debug=False):

        # TODO add/modify logic to select one of the available controllers
        current_controller : RobotController = self.controllers["approach"]
        # if approached to object ... then ... TODO
        # ... current_controller : RobotController = self.controllers["pickup"]


        # after selection of controller, forward data
        current_controller.update(robot, detections)

        
        