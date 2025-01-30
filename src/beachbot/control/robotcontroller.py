from dataclasses import dataclass
import threading
from typing import List, Union, Tuple

from beachbot.utils.properties import HasProperties


@dataclass
class BoxDef:
    left: float = 0.5
    top: float = 0.5
    w: float = 0.5
    h: float = 0.5
    class_name: str = "unknown"
    confidence : float = -1



class RobotController(HasProperties):
    def __init__(self):
        super().__init__()

        self.debug=False
        self.register_property("debug") 


    def update(self, robot, detections: List[BoxDef] = None):
        raise NotImplementedError()
    

