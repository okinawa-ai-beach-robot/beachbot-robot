import math
from typing import List
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.robotcontroller import RobotController, BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.utils.controllercollection import PIDController
from beachbot.config import logger


class SearchController(RobotController):
    def __init__(self):
        super().__init__()

        self.targetfilter=["toilet", "sports ball"]
        self.register_property("targetfilter", ",".join(self.targetfilter), descr="Explanation!!!")

        self.test=0.5
        self.register_property("test", descr="test var")

        self.test2=0.5
        self.register_property("test2", min_value=0.0, max_value=1.0, descr="test2 var")

        self.test3=True
        self.register_property("test3", descr="test3 var")
            

    def update(self, robot: RobotInterface, detections: List[BoxDef]=None) -> bool:
        """
        Look around and search for trash

        Args:
            robot (RobotInterface): Robot interface
            detections (List[BoxDef], optional): List of detections. Defaults to None.

        Returns:
            bool: True if controller has acheived target, False otherwise 
        """


        if self.debug:
            print(self.test)
            print(self.test2)
            print(self.test3)
            print(self.targetfilter)


        trash_to_follow: List[BoxDef] = []
        for det in detections:
            if det.class_name in self.targetfilter:
                trash_to_follow.append(det)



        if len(trash_to_follow)>0:
            return RESULT.SUCCESS
        else:
            robot.set_target_velocity(self.test, 0)
            return RESULT.BUSY

       


        return RESULT.BUSY
