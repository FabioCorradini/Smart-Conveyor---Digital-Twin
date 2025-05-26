from digtwin.gui.models.dt_models import DTStatefulModel, DTModelState
import numpy as np
from typing import Union
from digtwin.gui.dt_loadable import DTLoadable
from direct.showbase import DirectObject
import logging

_logger = logging.getLogger(__name__)

class SensorTrigger(DirectObject.DirectObject):
    def __init__(self, sensor_object: 'DTSensor'):
        _logger.info(f"Waiting for event into_{sensor_object.name}_0_collision")

        self.accept(f"into_{sensor_object.name}_0_collision", self._turn_on)
        self.accept(f"out_{sensor_object.name}_1_collision", self._turn_off)
        self.sensor_object = sensor_object

    def _turn_on(self, a):
        _logger.info(f"Turning on sensor {self.sensor_object.name}")
        self.sensor_object.state_id = 1
        self.sensor_object.to_state()

    def _turn_off(self, b):
        _logger.info(f"Turning on sensor {self.sensor_object.name}")
        self.sensor_object.state_id = 0
        self.sensor_object.to_state()




class DTSensor(DTStatefulModel):
    def __init__(self, name: str, parent: Union[str, None, 'DTLoadable'] = None):
        super().__init__(name, parent)
        self._sensor_trigger_handler = SensorTrigger(self)

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
            detectable = True
        else:
            detectable = False


        self.states.append(DTModelState(f"{self.name}_{num_id}", color, model_path, position, rotation_axis,
                                        rotation_angle, collision_center, collision_radius, collision_sides,
                                        collision_rotation_axis, collision_rotation_angle, False, False,
                                        detectable))

    @staticmethod
    def from_dict(data_dict: dict) -> 'DTSensor':
        obj = DTSensor(
            data_dict["name"],
            data_dict["parent"]
        )

        for state in data_dict["states"]:
            new_state = DTModelState.from_dict(state)
            obj.states.append(new_state)

        return obj
