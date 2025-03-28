from typing import List

from beachbot.config import logger
from beachbot.control.controllerselector import ControllerSelector
from beachbot.control.robotcontroller import CONTROLLERRESULT as RESULT
from beachbot.control.robotcontroller import BoxDef
from beachbot.control.approachdebris import ApproachDebris
from beachbot.robot.robotinterface import RobotInterface
from beachbot.control.potential_targets import potential_targets


class CustomApproachDebris(ApproachDebris):
    def __init__(self, parent=None):
        super().__init__(parent)

    def search(self, robot: RobotInterface):
        """
        Search when nothing visible

        Args:
            robot (RobotInterface): Robot interface
        """
        robot.set_target_velocity(0, 0)


class Hackathon(ControllerSelector):
    def __init__(self):
        approachDebris = CustomApproachDebris(self)
        super().__init__(approachDebris)

        # Task 1: Create a list of objects to be picked up in order:
        self.target_order = [
            potential_targets[0],
            potential_targets[1],
            potential_targets[2],
        ]

        # current target is the first target:
        self.current_target = self.target_order[0]
        logger.debug("ControllerSelector Starting with target " + self.current_target)

        # update the sub-controller to approach the current target
        self.approachDebris.targetfilter = self.current_target

    def update(self, robot: RobotInterface, detections: List[BoxDef] = None):
        # 1.) If current control mode if approaching
        if self.controller is self.approachDebris:

            # execute controller and test if it completed it's task:
            if self.controller.update(robot, detections) == RESULT.SUCCESS:

                # select pickup controller as next controller:
                self.controller = self.pickup

        # 2.) If the current control mode if pick-up
        elif self.controller is self.pickup:

            # execute controller pick-up and test if it is still busy with picking the object
            if self.controller.update(robot, detections) != RESULT.BUSY:

                # Check if current_target no longer visible (not visible=picked up and placed in basked, if still visible it was dropped!)
                if not self.target_visible(detections):

                    self.next_target()

                # if picking up failed or succeeded, continue with approaching objects again next!
                self.controller = self.approachDebris

        return RESULT.BUSY
