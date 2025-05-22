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
                 rotation_angle = np.zeros(1)
                 ):

        super().__init__(name, model_path, parent, color, position, rotation_axis, rotation_angle)
