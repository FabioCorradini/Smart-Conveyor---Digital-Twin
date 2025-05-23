import json
from pathlib import Path
from typing import Union, Any

from panda3d.core import (Filename, NodePath, Loader, CollisionNode, CollisionSphere, CollisionBox, Point3,
                          CollisionSolid, BitMask32)
import logging
from digtwin.gui.dt_loadable import DTLoadable
import numpy as np

from digtwin.gui.nodes.dt_nodes import DTNode

_logger = logging.getLogger(__name__)

class DTModel(DTLoadable):
    model_path: str | None
    parent: str | None

    def __init__(self,
                 name: str,
                 model_path: str,
                 parent:  Union[str, None, 'DTLoadable'] = None,
                 color = None,
                 position = np.zeros(3),
                 rotation_axis = np.zeros(3),
                 rotation_angle = np.zeros(1),
                 collision_center: np.ndarray | None = None,
                 collision_radius = 0.0,
                 collision_sides:  tuple[float, float, float] = (0.0, 0.0, 0.0)
                 ):

        super().__init__(name, parent)
        self.color = color
        self.model_path = model_path
        self.current_position = position
        self.current_rotation_axis = rotation_axis
        self.current_rotation_angle = rotation_angle
        self.collision_center = collision_center
        self.collision_radius = collision_radius
        self.collision_sides = collision_sides

    @property
    def model_path(self) -> Filename:
        return Filename.from_os_specific(str(self._model_path))

    @model_path.setter
    def model_path(self, model_path: str | Filename | Path):
        if not isinstance(model_path, Path):
            self._model_path = Path(str(model_path))

        if not self._model_path.is_file():
            _logger.warning(f"Model file {self._model_path} does not exist")

    def to_dict(self) -> dict:
        data_dict = super().to_dict()
        data_dict["model_path"] = str(self.model_path)
        data_dict["position"] = self.current_position.tolist()
        data_dict["rotation_axis"] = self.current_rotation_axis.tolist()
        data_dict["rotation_angle"] = self.current_rotation_angle.tolist()
        data_dict["collision_center"] = self.collision_center.tolist() if self.collision_center is not None else None
        data_dict["collision_radius"] = self.collision_radius
        data_dict["collision_sides"] = self.collision_sides
        return data_dict

    @staticmethod
    def from_dict(data_dict: dict) -> 'DTModel':
        return DTModel(
            data_dict["name"],
            data_dict["model_path"],
            data_dict["parent"],
            data_dict["color"] if "color" in data_dict else None,
            np.array(data_dict["position"]),
            np.array(data_dict["rotation_axis"]),
            np.array(data_dict["rotation_angle"]),
            np.array(data_dict["collision_center"]) if data_dict["collision_center"] is not None else None,
            data_dict["collision_radius"] if "collision_radius" in data_dict else 0.0,
            data_dict["collision_sides"] if "collision_sides" in data_dict else (0.0, 0.0, 0.0)
        )

    def build_collision_solid(self) -> CollisionSolid:
        if self.collision_radius > 0:
            return CollisionSphere(
                self.collision_center[0],
                self.collision_center[1],
                self.collision_center[2],
                self.collision_radius)
        elif self.collision_sides[0] > 0 or self.collision_sides[1] > 0 or self.collision_sides[2] > 0:
            return CollisionBox(
                Point3(self.collision_center[0], self.collision_center[1], self.collision_center[2]),
                self.collision_sides[0],
                self.collision_sides[1],
                self.collision_sides[2]
            )
        else:
            raise ValueError("No valid collision solid found")


class DTStatefulModel(DTLoadable):
    states: list['DTModelState']
    _old_reference: NodePath | None

    def __init__(self, name: str, parent: Union[str, None, 'DTLoadable'] = None):
        super().__init__(name, parent)
        self.states = []
        self._current_state_id = 0
        self._old_reference = None

    def __len__(self) -> int:
        return len(self.states)

    @property
    def state_id(self) -> int:
        return self._current_state_id

    @state_id.setter
    def state_id(self, state_id: int):
        if state_id < len(self.states):
            self._old_reference = self.node_path_reference
            self._current_state_id = state_id
        else:
            _logger.error(f"State id {state_id} does not exist for model {self.name}")

    def to_state(self):
        self.to_color()
        self.to_position()
        if self._old_reference != self.node_path_reference:
            if self._old_reference is not None and not self._old_reference.is_stashed():
                self._old_reference.stash()
            if self.node_path_reference.is_stashed():
                self.node_path_reference.unstash()


    @property
    def current_position(self):
        return self.states[self._current_state_id].current_position

    @current_position.setter
    def current_position(self, position):
        return

    @property
    def current_rotation_axis(self):
        return self.states[self._current_state_id].current_rotation_axis

    @current_rotation_axis.setter
    def current_rotation_axis(self, rotation_axis):
        return

    @property
    def current_rotation_angle(self):
        return self.states[self._current_state_id].current_rotation_angle

    @current_rotation_angle.setter
    def current_rotation_angle(self, rotation_angle):
        return

    @property
    def node_path_reference(self):
        return self.states[self._current_state_id].node_path_reference

    @node_path_reference.setter
    def node_path_reference(self, node_path_reference):
        return

    @property
    def color(self):
        return self.states[self._current_state_id].color

    @color.setter
    def color(self, color):
        return

    def add_state(self,
                  model_path=None,
                  color=None,
                  position=np.zeros(3),
                  rotation_axis=np.zeros(3),
                  rotation_angle=np.zeros(1),
                  collision_center: np.ndarray | None= None,
                  collision_radius=0.0,
                  collision_sides: tuple[float, float, float] = (0.0, 0.0, 0.0)
                  ):

        num_id = len(self.states)
        if num_id and model_path is None:
                _logger.error(f"Non null model path required for the first state")
                return

        if model_path is None:
            model_path = self.states[num_id-1].model_path


        self.states.append(DTModelState(num_id, color, model_path, position, rotation_axis,
                                        rotation_angle, collision_center, collision_radius, collision_sides))

    def populate(self, loader: Loader) -> None:
        for state in self.states:
            state.node_path_reference = loader.load_model(state.model_path)
            state.node_path_reference.name = f"{self.name}_{state.name}"

    def to_dict(self) -> dict:
        data_dict = super().to_dict()
        states_list = [state.to_dict() for state in self.states]
        data_dict["states"] = states_list
        return data_dict

    @staticmethod
    def from_dict(data_dict: dict) -> 'DTStatefulModel':
        obj = DTStatefulModel(
            data_dict["name"],
            data_dict["parent"]
        )

        for state in data_dict["states"]:
            new_state = DTModelState.from_dict(state)
            obj.states.append(new_state)

        return obj

    def reparent(self, parent: NodePath):
        for state in self.states:
            if state.node_path_reference is not None:
                state.node_path_reference.reparentTo(parent)
                if state.collision_center is not None:
                    c_node = CollisionNode("collision")
                    try:
                        c_node.addSolid(state.build_collision_solid())
                    except ValueError as e:
                        _logger.error(f"Failed to build collision ({self.name}_{state.name}) : {e}")
                        raise e
                    c_node.setCollideMask(BitMask32(0x01))
                    c_node_np = state.node_path_reference.attach_new_node(c_node)
                    c_node_np.setTag("clickable", "1")

                state.node_path_reference.stash()
            else:
                _logger.warning(f"Tried to reparent uninitialized state {self.name}_{state.num_id} to parent {parent}")



class DTModelState(DTModel):
    def __init__(self,
                 num_id: int,
                 color: tuple[float, float, float, float] | None,
                 model_path: str,
                 position=np.zeros(3),
                 rotation_axis=np.zeros(3),
                 rotation_angle=np.zeros(1),
                 collision_center: np.ndarray | None = None,
                 collision_radius: float = 0.0,
                 collision_sides: tuple[float, float, float] = (0.0, 0.0, 0.0)
                 ):

        super().__init__(
            name = str(num_id),
            model_path = model_path,
            color = color,
            position = position,
            rotation_axis = rotation_axis,
            rotation_angle = rotation_angle,
            collision_center = collision_center,
            collision_radius = collision_radius,
            collision_sides = collision_sides
        )




