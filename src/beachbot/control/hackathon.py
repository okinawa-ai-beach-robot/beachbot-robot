from beachbot.control.controllerselector import ControllerSelector
from beachbot.config import logger
from beachbot.robot.robotinterface import RobotInterface
from typing import List
from beachbot.control.robotcontroller import BoxDef
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT

class Hackathon(ControllerSelector):
    def __init__(self):
        super().__init__()


        # list of objects to be picked up:
        self.target_order = ["bottle", "cup", "sports ball"]

        # current target is the first target:
        self.current_target = self.target_order[0]
        logger.debug("ControllerSelector Starting with target " + self.current_target)

        # update the sub-controller to approach the current target
        self.approachDebris.targetfilter = self.current_target

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        if self.controller is self.approachDebris:
            # 1.) If current control mode if approaching

            # execute controller and test if it completed it's task:
            if self.controller.update(robot, detections) == RESULT.SUCCESS:

                # print debug info in console:
                if self.debug:
                    logger.info(f"Object {self.current_target} approached, will try to pick it up!")

                # select pickup controller as next controller:
                self.controller = self.pickup

        elif self.controller is self.pickup:
            # 2.) If the current control mode if pick-up

            # execute controller pick-up and test if it is still busy with picking the object
            if self.controller.update(robot, detections) != RESULT.BUSY:
                # if picking up is completed or failed

                # Check if current_target no longer visible (not visible=picked up and placed in basked, if still visible it may dropped!)
                if not self.target_visible(detections):

                    # Bottle disappeared! Consider this target successfully picked up
                    self.next_target()
                
                # if picking up failed or succeeded, continue with approaching objects again next!
                self.controller = self.approachDebris
                if self.debug:
                    logger.debug(f"Pickup done, next we try to approach the object {self.approachDebris.targetfilter }")

        return RESULT.BUSY

