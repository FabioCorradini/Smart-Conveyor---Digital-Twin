from digtwin.gui.models.dt_models import DTModel
from typing import Union
import numpy as np


class DTActor(DTModel):
    def __init__(self,
                 name: str,
                 model_path: str,
                 parent:  Union[str, None, 'DTLoadable'] = None,
                 color = None,
                 position = np.zeros(3),
                 rotation_axis = np.zeros(3),
                 rotation_angle = np.zeros(1),
                 collision_center: np.ndarray | None = None,
                 collision_radius=0.0,
                 collision_sides: tuple[float, float, float] = (0.0, 0.0, 0.0)
                 ):

        super().__init__(name, model_path, parent, color, position, rotation_axis, rotation_angle,
                         collision_center, collision_radius, collision_sides)
