import numpy as np
import logging
from digtwin.gui.dt_loadable import DTLoadable

_logger = logging.getLogger(__name__)


class DTNode(DTLoadable):
    _last_theta: None | np.ndarray

    def __init__(self,
                 name: str,
                 parent: str | None | DTLoadable = None,
                 theta_period: np.ndarray | None = None,
                 delta_theta: np.ndarray | None = None,
                 n_var: int = 1
                 ):
        super().__init__(name, parent)
        self.node_states: list[DTNodeState] = []
        self._n_var = n_var
        self.current_position = np.zeros(3)
        self.current_rotation_axis = np.zeros(3)
        self.current_rotation_angle = np.zeros(1)
        self.theta_period = theta_period
        self.current_state_id = 0
        self._last_theta = None
        self.fixed = True
        self.delta_theta = delta_theta if delta_theta is not None else np.zeros((1, n_var))


    def set_theta(self, theta: tuple[float]):
        assert len(theta) == self._n_var

        if self.theta_period is not None:
            current_theta = (np.array(theta).transpose() + self.delta_theta) % self.theta_period
            # print(f"current theta: {current_theta}, last theta: {self._last_theta}, state_id: {self.current_state_id}")

            if self.current_state_id < (len(self.node_states) - 1) or self._last_theta is None:
                if (current_theta > self.node_states[self.current_state_id].exit_theta_condition).any():
                    self.current_state_id += 1
                    # _logger.info(f"node {self.name} entering state {self.current_state_id}")
            else: # in the last state we look for resetting
                if (current_theta < self._last_theta).any():
                    self.current_state_id = 0
                    # _logger.info(f"node {self.name} entering state {self.current_state_id}")

            self._last_theta = current_theta

        else:
            current_theta = np.array(theta).transpose() + self.delta_theta

        current_state = self.node_states[self.current_state_id]

        if self.current_state_id > 0:
            current_theta = current_theta - self.node_states[self.current_state_id-1].exit_theta_condition  # theta relative to the current state

        self.current_position = current_state.get_shift(current_theta).flatten()
        self.current_rotation_axis = current_state.get_rotation_axis(current_theta).flatten()
        self.current_rotation_angle = current_state.get_rotation_angle(current_theta).flatten()

    def add_node_state(self,
                       state_name: str,
                       exit_theta_condition=np.zeros((1, 1)),
                       const_shift_matrix=np.zeros((3, 1)),
                       var_shift_matrix=np.zeros((3, 1)),
                       const_rotation_axis_matrix=np.zeros((3, 1)),
                       var_rotation_axis_matrix=np.zeros((3, 1)),
                       const_rotation_angle_matrix=np.zeros((1, 1)),
                       var_rotation_angle_matrix=np.zeros((1, 1))
                       ):

        if self.theta_period is not None or not bool(var_shift_matrix.all()) or not bool(var_rotation_axis_matrix.all()) or not bool(var_rotation_angle_matrix.all()):
            self.fixed = False  # if variables are not 0 the object is not fixed


        self.node_states.append(DTNodeState(
            state_name,
            exit_theta_condition,
            const_shift_matrix,
            var_shift_matrix,
            const_rotation_axis_matrix,
            var_rotation_axis_matrix,
            const_rotation_angle_matrix,
            var_rotation_angle_matrix,
            self._n_var
        ))

    def to_dict(self) -> dict:
        data_dict = super().to_dict()
        data_dict["node_states"] = [state.to_dict() for state in self.node_states]
        data_dict["theta_period"] = self.theta_period.tolist() if self.theta_period is not None else None
        data_dict["n_var"] = self._n_var
        data_dict["delta_theta"] = self.delta_theta.tolist()
        return data_dict

    @staticmethod
    def from_dict(data_dict: dict) -> 'DTNode':
        node = DTNode(data_dict["name"],
                      data_dict["parent"],
                      np.array(data_dict["theta_period"]) if data_dict["theta_period"] is not None else None,
                      np.array(data_dict["delta_theta"]),
                      data_dict["n_var"])

        node.color = data_dict["color"] if "color" in data_dict else None

        for state_dict in data_dict["node_states"]:
            new_state = DTNodeState.from_dict(state_dict)
            node.node_states.append(new_state)
        return node


class DTNodeState:
    def __init__(self,
                 state_name: str,
                 exit_theta_condition= np.zeros((1, 1)),
                 const_shift_matrix=np.zeros((3, 1)),
                 var_shift_matrix=np.zeros((3, 1)),
                 const_rotation_axis_matrix=np.zeros((3, 1)),
                 var_rotation_axis_matrix=np.zeros((3, 1)),
                 const_rotation_angle_matrix=np.zeros((1, 1)),
                 var_rotation_angle_matrix=np.zeros((1, 1)),
                 n_var:int = 1):
        """
        :param state_name:
        :param n_var: number of independent variables
        """

        self.state_name = state_name
        self.exit_theta_condition = exit_theta_condition

        self.const_shift_matrix = const_shift_matrix
        self.var_shift_matrix = var_shift_matrix

        self.const_rotation_axis_matrix = const_rotation_axis_matrix
        self.var_rotation_axis_matrix = var_rotation_axis_matrix

        self.const_rotation_angle_matrix = const_rotation_angle_matrix
        self.var_rotation_angle_matrix = var_rotation_angle_matrix

        self._n_var = n_var

    @property
    def state_name(self):
        return self._state_name

    @state_name.setter
    def state_name(self, name: str):
        assert name.isalnum()
        self._state_name = name

    def get_shift(self, theta: np.ndarray) -> np.ndarray:
        return self.const_shift_matrix + np.matmul(self.var_shift_matrix, theta)

    def get_rotation_axis(self, theta: np.ndarray) -> np.ndarray:
        return self.const_rotation_axis_matrix + np.matmul(self.var_rotation_axis_matrix, theta)

    def get_rotation_angle(self, theta: np.ndarray) -> np.ndarray:
        return self.const_rotation_angle_matrix + np.matmul(self.var_rotation_angle_matrix, theta)

    def _check_values(self):
        theta = np.ones((self._n_var,1))
        self.get_shift(theta)
        self.get_rotation_axis(theta)
        self.get_rotation_angle(theta)


    def to_dict(self):
        self._check_values()
        return {
            "state_name": self.state_name,
            "enter_theta_condition": self.exit_theta_condition.tolist(),
            "const_shift_matrix": self.const_shift_matrix.tolist(),
            "var_shift_matrix": self.var_shift_matrix.tolist(),
            "const_rotation_axis_matrix": self.const_rotation_axis_matrix.tolist(),
            "var_rotation_axis_matrix": self.var_rotation_axis_matrix.tolist(),
            "const_rotation_angle_matrix": self.const_rotation_angle_matrix.tolist(),
            "var_rotation_angle_matrix": self.var_rotation_angle_matrix.tolist(),
            "n_var" : self._n_var,
        }

    @staticmethod
    def from_dict(data: dict):
        return DTNodeState(
            data["state_name"],
            np.array(data["enter_theta_condition"]),
            np.array(data["const_shift_matrix"]),
            np.array(data["var_shift_matrix"]),
            np.array(data["const_rotation_axis_matrix"]),
            np.array(data["var_rotation_axis_matrix"]),
            np.array(data["const_rotation_angle_matrix"]),
            np.array(data["var_rotation_angle_matrix"]),
            data["n_var"],
        )

    def __str__(self):
        return f"{self.state_name}"








        