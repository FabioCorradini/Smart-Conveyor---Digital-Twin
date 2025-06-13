import json
from pathlib import Path
from typing import Union, Any

from panda3d.core import (Filename, NodePath, Loader, CollisionNode, CollisionSphere, CollisionBox, Point3,
                          CollisionSolid, BitMask32, LQuaternion, LVector3f)
import logging
from digtwin.gui.dt_loadable import DTLoadable
import numpy as np

from digtwin.gui.nodes.dt_nodes import DTNode

_logger = logging.getLogger(__name__)

class DTModel(DTLoadable):
    collision_node_reference: None | NodePath
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
                 collision_sides:  tuple[float, float, float] = (0.0, 0.0, 0.0),
                 collision_rotation_axis =  np.zeros(3),
                 collision_rotation_angle = 0.0,
                 clickable: bool = False,
                 solid: bool = False,
                 detectable: bool = False
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
        self.collision_rotation_axis = collision_rotation_axis
        self.collision_rotation_angle = collision_rotation_angle
        self.clickable = clickable
        self.solid = solid
        self.detectable = detectable
        self.collision_node_reference = None

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
        data_dict["collision_rotation_axis"] = self.collision_rotation_axis.tolist()
        data_dict["collision_rotation_angle"] = self.collision_rotation_angle
        data_dict["clickable"] = self.clickable
        data_dict["solid"] = self.solid
        data_dict["detectable"] = self.detectable
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
            data_dict["collision_sides"] if "collision_sides" in data_dict else (0.0, 0.0, 0.0),
            np.array(data_dict["collision_rotation_axis"]),
            data_dict["collision_rotation_angle"],
            data_dict["clickable"],
            data_dict["solid"],
            data_dict["detectable"]
        )

    def build_collision_solid(self) -> CollisionSolid:
        if self.collision_radius > 0:
            solid = CollisionSphere(
                0.0,
                0.0,
                0.0,
                self.collision_radius)
        elif self.collision_sides[0] > 0 or self.collision_sides[1] > 0 or self.collision_sides[2] > 0:
            solid = CollisionBox(
                Point3(0.0, 0.0, 0.0),
                self.collision_sides[0],
                self.collision_sides[1],
                self.collision_sides[2]
            )
        else:
            raise ValueError("No valid collision solid found")

        if self.detectable:
            solid.set_tangible(False)

        return solid


    def align_collision_solid(self, collision_node_path: NodePath):
        if self.collision_radius <= 0: # if is a sphere alignment is not done
            q = LQuaternion()
            if self.collision_rotation_axis.any():
                q.set_from_axis_angle_rad(self.collision_rotation_angle,
                                          LVector3f(*self.collision_rotation_axis.tolist()))
                collision_node_path.set_quat(q)
        collision_node_path.set_pos(*self.collision_center.tolist())


    def reparent(self, parent: NodePath):
        super().reparent(parent)
        self.set_collision()

    def set_collision(self):
        if self.node_path_reference is not None:
            if self.collision_center is not None:
                c_node = CollisionNode(f"{self.name}_collision")
                try:
                    c_node.addSolid(self.build_collision_solid())
                except ValueError as e:
                    _logger.error(f"Failed to build collision ({self.name}) : {e}")
                    raise e
                bit_mask = 0x00
                if self.clickable:
                    bit_mask |= 0x01
                if self.solid:
                    bit_mask |= 0x02
                if self.detectable:
                    bit_mask |= 0x04

                c_node.setCollideMask(BitMask32(bit_mask))
                c_node_np = self.node_path_reference.attach_new_node(c_node)
                c_node_np.show()
                self.align_collision_solid(c_node_np)
                self.collision_node_reference = c_node_np
                if self.clickable:
                    c_node_np.setTag("clickable", "1")
        else:
            _logger.warning(f"Tryng to set collision to uninit solid {self.name}")



class DTStatefulModel(DTLoadable):
    states: list['DTModelState']
    _old_reference: NodePath | None

    def __init__(self, name: str,source_dev: str, parent: Union[str, None, 'DTLoadable'] = None):
        super().__init__(name, parent)
        self.states = []
        self._current_state_id = 0
        self._old_reference = None
        self.source_dev = source_dev

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
                  collision_sides: tuple[float, float, float] = (0.0, 0.0, 0.0),
                  collision_rotation_axis =  np.zeros(3),
                  collision_rotation_angle = 0.0
                  ):

        num_id = len(self.states)
        if num_id and model_path is None:
                _logger.error(f"Non null model path required for the first state")
                return

        if model_path is None:
            model_path = self.states[num_id-1].model_path

        if collision_center is not None:
            clickable = True
        else:
            clickable = False


        self.states.append(DTModelState(f"{self.name}_{num_id}", color, model_path, position, rotation_axis,
                                        rotation_angle, collision_center, collision_radius, collision_sides,
                                        collision_rotation_axis, collision_rotation_angle, clickable))

    def populate(self, loader: Loader) -> None:
        for state in self.states:
            state.node_path_reference = loader.load_model(state.model_path)
            state.node_path_reference.name = state.name

    def to_dict(self) -> dict:
        data_dict = super().to_dict()
        states_list = [state.to_dict() for state in self.states]
        data_dict["states"] = states_list
        data_dict["source_dev"] = self.source_dev
        return data_dict

    @staticmethod
    def from_dict(data_dict: dict) -> 'DTStatefulModel':
        obj = DTStatefulModel(
            data_dict["name"],
            data_dict["source_dev"],
            data_dict["parent"]
        )

        for state in data_dict["states"]:
            new_state = DTModelState.from_dict(state)
            obj.states.append(new_state)

        return obj

    def reparent(self, parent: NodePath):
        for state in self.states:
            if state.node_path_reference is not None:
                state.reparent(parent)
                state.node_path_reference.stash()
            else:
                _logger.warning(f"Tried to reparent uninitialized state {self.name}_{state.num_id} to parent {parent}")



class DTModelState(DTModel):
    def __init__(self,
                 name: str,
                 color: tuple[float, float, float, float] | None,
                 model_path: str,
                 position=np.zeros(3),
                 rotation_axis=np.zeros(3),
                 rotation_angle=np.zeros(1),
                 collision_center: np.ndarray | None = None,
                 collision_radius: float = 0.0,
                 collision_sides: tuple[float, float, float] = (0.0, 0.0, 0.0),
                 collision_rotation_axis =  np.zeros(3),
                 collision_rotation_angle = 0.0,
                 clickable: bool = False,
                 solid: bool = False,
                 detectable: bool = False,
                 ):

        super().__init__(
            name = name,
            model_path = model_path,
            color = color,
            position = position,
            rotation_axis = rotation_axis,
            rotation_angle = rotation_angle,
            collision_center = collision_center,
            collision_radius = collision_radius,
            collision_sides = collision_sides,
            collision_rotation_axis= collision_rotation_axis,
            collision_rotation_angle=collision_rotation_angle,
            clickable= clickable,
            solid= solid,
            detectable=detectable
        )




