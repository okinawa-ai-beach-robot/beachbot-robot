from dataclasses import dataclass
import threading
from typing import List, Union


@dataclass
class BoxDef:
    left: float = 0.5
    top: float = 0.5
    w: float = 0.5
    h: float = 0.5
    class_name: str = "unknown"
    confidence : float = -1

@dataclass
class RobotControllerProperty:
    value: Union[bool, float, str]
    default: Union[bool, float, str]
    max: Union[float,None] = None
    min: Union[float,None] = None


class RobotController:
    def __init__(self):
        self._properties:dict[str, RobotControllerProperty]={}
        self._properties_lock = threading.Lock()


    def _register_property(self, name:str, default_value:Union[bool, float, str], max_value:Union[float,None] = None, min_value: Union[float,None] = None):
        prop = RobotControllerProperty(value=default_value, default=default_value, max=max_value, min=min_value)
        self._properties[name]=prop

    def set_property(self, name:str, value:Union[bool, float, str]):
        was_updated=False
        with self._properties_lock:
            if name in self._properties:
                assert type(self._properties[name].default)==type(value), "Error: Property type mismatch"
                was_updated=self._properties[name].value!=value
                self._properties[name].value=value
                
                # Bounds check only if float:
                if type(self._properties[name].value)==float:
                    if self._properties[name].max is not None and self._properties[name].max<self._properties[name].value:
                        self._properties[name].value=self._properties[name].max
                    if self._properties[name].max is not None and self._properties[name].min>self._properties[name].value:
                        self._properties[name].value=self._properties[name].min
        if was_updated:
            self._property_changed_callback(name)

    def get_property(self,name:str) -> Union[bool, float, str, None]:
        with self._properties_lock:
            if name not in self._properties:
                return None
            return self._properties[name].value
    def get_property_bounds(self, name:str) -> Union[tuple[Union[float,None], Union[float,None]], None]:
        with self._properties_lock:
            if name not in self._properties:
                return None
            return (self._properties[name].min, self._properties[name].max)
        
    def list_property_names(self)->list[str]:
        with self._properties_lock:
            return list(self._properties.keys())

        
    def _property_changed_callback(self, name:str):
        raise ValueError(f"Unknown Parameter {name}")


    def update(self, robot, detections: List[BoxDef] = None):
        raise NotImplementedError()
    

