import json
from pathlib import Path
from typing import Union
import numpy as np
from panda3d.core import NodePath, LQuaternion, LVector3f, Material, LVecBase4f

class DTLoadable:
    """
    Abstract class that loads and save data from JSON files
    """
    node_path_reference: NodePath | None = None
    color: tuple[float, float, float, float] | None

    def __init__(self, name: str, parent: Union[str, None, 'DTLoadable'] = None):
        self.name = name
        self.parent = parent
        self.current_position = np.zeros(3)
        self.current_rotation_axis = np.zeros(3)
        self.current_rotation_angle = np.zeros(1)
        self.node_path_reference = None
        self.color = None

    @property
    def parent(self) -> str | None:
        return self._parent

    @parent.setter
    def parent(self, value: Union[str, None, 'DTLoadable']):
        if not isinstance(value, DTLoadable):
            self._parent = value
        else:
            self._parent = value.name

    @staticmethod
    def from_dict(data_dict: dict)  -> 'DTLoadable':
        model = DTLoadable(data_dict['name'], data_dict['parent'])
        model.color = data_dict['color'] if 'color' in data_dict else None
        return model

    def to_dict(self) -> dict:
        return {"name": self.name, "parent": self.parent, "color": self.color}

    def copy(self, new_name: str | None = None) -> 'DTLoadable':
        data_dict = self.to_dict()
        if new_name is not None:
            data_dict['name'] = new_name
        return self.from_dict(data_dict)

    @classmethod
    def load(cls, path: Path) -> 'DTLoadable':
        data_dict = json.load(path.open())
        return cls.from_dict(data_dict)

    def save(self, path_dir: Path) -> None:

        file_path = path_dir / f"{self.name}.json"

        json.dump(self.to_dict(), file_path.open("w"))

    def _get_rotation(self)->LQuaternion:
        q = LQuaternion()
        if self.current_rotation_axis.any():
            q.set_from_axis_angle_rad(self.current_rotation_angle[0], LVector3f(*self.current_rotation_axis.tolist()))

        return q

    def reparent(self, parent: NodePath):
        if self.node_path_reference is not None:
            self.node_path_reference.reparentTo(parent)


    def to_position(self):
        if self.node_path_reference is not None:
            self.node_path_reference.set_pos_quat(LVector3f(*self.current_position) , self._get_rotation())

    def to_color(self):
        if self.node_path_reference is not None:
            if self.color is not None:
                mat = Material()
                mat.set_ambient(LVecBase4f(1, 1, 1, 1.0))
                mat.set_diffuse(LVecBase4f(1, 1, 1, 1.0))
                # print(f"Cor set to {self.color} for object {self.name}")
                # mat.set_shininess(64)
                self.node_path_reference.set_material(mat)
                self.node_path_reference.set_color_scale(*self.color)
            else:
                self.node_path_reference.clear_color_scale()

