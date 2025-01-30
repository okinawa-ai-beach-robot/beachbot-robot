from __future__ import annotations
from dataclasses import dataclass
import threading
from typing import List, Union, Tuple


@dataclass
class Property:
    value: Union[bool, float, str]
    default: Union[bool, float, str]
    max: Union[float,None] = None
    min: Union[float,None] = None

class HasProperties(object):
    def __init__(self):
        self._properties_lock = threading.Lock()
        self.clear_properties()



    def clear_properties(self):
        with self._properties_lock:
            self._properties_children = {}
            self._properties:dict[str, Property]={}

        

    
    def register_property(self, name:str, default_value:Union[bool, float, str, None]=None, max_value:Union[float,None] = None, min_value: Union[float,None] = None):
        if default_value is None:
            try:
                default_value = getattr(self, name)
            except AttributeError:
                raise ValueError(f"Default property not given, assume self.{name} exists as default")
        prop = Property(value=default_value, default=default_value, max=max_value, min=min_value)
        self._properties[name]=prop


    def register_child_properties(self, class_instance : HasProperties, class_name : str):
        # Collect all properties of the given subclass and add to this class
        for name_property in class_instance.list_property_names():
            prop_default_val = class_instance.get_property(name_property)
            prop_bounds = class_instance.get_property_bounds(name_property)
            prop_min = None
            prop_max = None
            if prop_bounds is not None:
                prop_min, prop_max = prop_bounds
            self._properties_children[class_name] = class_instance
            self.register_property(class_name + "." + name_property, prop_default_val, max_value=prop_max, min_value=prop_min)





    def set_property(self, name:str, value:Union[bool, float, str]):
        if "." in name:
            # forward property change to children ....
            childname, propname=  name.split(".", 1)
            if childname in self._properties_children:
                self._properties_children[childname].set_property(propname, value)
            else:
                raise ValueError(f"Child property module {childname} not known, can not set property {propname}")
        else:
            was_updated=False
            with self._properties_lock:
                if name in self._properties:
                    assert type(self._properties[name].default)==type(value), f"Error: Property type mismatch {type(self._properties[name].default)} {type(value)}"
                    was_updated=self._properties[name].value!=value
                    self._properties[name].value=value
                    
                    # Bounds check only if float:
                    if type(self._properties[name].value)==float:
                        if self._properties[name].max is not None and self._properties[name].max<self._properties[name].value:
                            self._properties[name].value=self._properties[name].max
                        if self._properties[name].max is not None and self._properties[name].min>self._properties[name].value:
                            self._properties[name].value=self._properties[name].min
            if was_updated:
                self.property_changed_callback(name)


    def get_property(self,name:str) -> Union[bool, float, str, None]:
        if "." in name:
            # retrieve property values from children ....
            childname, propname=  name.split(".", 1)
            if childname in self._properties_children:
                return self._properties_children[childname].get_property(propname)
            else:
                raise ValueError(f"Child property module {childname} not known, can not get property {propname}")
        else:
            with self._properties_lock:
                if name not in self._properties:
                    return None
                return self._properties[name].value
    
    def get_property_bounds(self, name:str) -> Union[Tuple[Union[float,None], Union[float,None]], None]:
        if "." in name:
            # retrieve property values from children ....
            childname, propname=  name.split(".", 1)
            if childname in self._properties_children:
                return self._properties_children[childname].get_property_bounds(propname)
            else:
                raise ValueError(f"Child property module {childname} not known, can not get property bounds of property {propname}")
        else:
            with self._properties_lock:
                if name not in self._properties:
                    return None
                return (self._properties[name].min, self._properties[name].max)




    def list_property_names(self)->List[str]:
        with self._properties_lock:
            return list(self._properties.keys())
        




    def property_changed_callback(self, name:str):
        try:
            setattr(self, name, self.get_property(name))
        except AttributeError:
            raise ValueError(f"Unknown Parameter stored in self.{name}")