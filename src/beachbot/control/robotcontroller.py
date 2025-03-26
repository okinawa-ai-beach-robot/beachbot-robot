from dataclasses import dataclass
from typing import List
from enum import Enum

from beachbot.utils.properties import HasProperties


@dataclass
class BoxDef:
    left: float = 0.5
    top: float = 0.5
    w: float = 0.5
    h: float = 0.5
    class_name: str = "unknown"
    confidence: float = -1


class CONTROLLERRESULT(Enum):
    SUCCESS = 1
    FAILURE = 2
    BUSY = 3


class RobotController(HasProperties):
    def __init__(self, parent=None):
        super().__init__()

        self.debug = False
        self.register_property("debug")

    def update(self, robot, detections: List[BoxDef] = None) -> CONTROLLERRESULT:
        raise NotImplementedError()
