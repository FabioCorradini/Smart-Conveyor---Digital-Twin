import numpy as np
from queue import Queue, Full
from dataclasses import dataclass
from enum import Enum
import logging

from matplotlib.lines import lineStyles

_logger = logging.getLogger(__name__)

class CommandType(Enum):
    MOVEMENT_COMMAND = "MOVEMENT_COMMAND"
    ROTATE_COMMAND = "ROTATE_COMMAND"
    PADDLE_COMMAND = "PADDLE_COMMAND"
    ALIGN_COMMAND = "ALIGN_COMMAND"

@dataclass
class Command:
    command_type: CommandType
    value: tuple
    submitted: bool = False
    done: bool = False

class Robot:
    _current_command: None | Command

    def __init__(self):
        self._current_x_position = 0.0
        self._current_y_position = 0.0
        self._current_angle = 0.0

        self._target_x_position = 0.0
        self._target_y_position = 0.0
        self._target_angle = 0.0

        self.shift_speed = 50.0       # mm/s
        self.rot_speed = np.pi / 2  # 90°/s
        self.paddle_speed = 5.0       # mm/s

        self._delta_vector = np.zeros(2)
        self._delta_ang = 0.0
        self._delta_paddle = 0.0

        self._x_speed = 0.0
        self._y_speed = 0.0
        self._r_speed = 0.0
        self._z_speed = 0.0

        self._current_paddle_position = 0.0
        self._last_paddle_position = 0.0
        self._target_paddle_position = 0.0



        self._last_time = None

        self._command_queue = Queue(maxsize=20)

        self._paused = False

        self.error_string = ""
        self.full_queue = False

        self._current_command = None


    def command_queue_len(self):
        return self._command_queue.qsize()

    def is_busy(self):
        return not( (self._current_command is None or self._current_command.done) and  self._command_queue.empty())


    def _enqueue(self, command: Command):
        try:
            self._command_queue.put(command, block=False)
        except Full:
            self.error_string += "Command queue is full."
            self.full_queue = True

    def reset_alarms(self):
        self.error_string = ""
        self.full_queue = False

    def move_to(self, x: float, y: float):
        self._enqueue(Command(CommandType.ALIGN_COMMAND, (x, y)))
        self._enqueue(Command(CommandType.MOVEMENT_COMMAND, (x, y)))

    def rotate(self, r: float):
        self._enqueue(Command(CommandType.ROTATE_COMMAND, (r,)))

    def move_paddle(self, z: float):
        self._enqueue(Command(CommandType.PADDLE_COMMAND, ( min(max(z, 0.0), 50.0),)))

    def get_robot_theta(self) -> tuple[float, float, float]:
        return self._current_x_position, self._current_y_position, self._current_angle

    def get_robot_position(self) -> list[float]:
        return [self._current_x_position, self._current_y_position]

    def get_robot_angle(self) -> float:
        return self._current_angle

    def get_paddle_theta(self) -> float:
        return self._current_paddle_position

    def pause(self):
        self._paused = True

    def resume(self):
        self._paused = False

    def is_paused(self):
        return self._paused

    def stop(self):
        self._paused = True

        if self._current_command is not None:
            self._current_command.done = True

        while not self._command_queue.empty():
            self._command_queue.get(block=False)

        self._paused = False

    def get_state(self) -> str:
        if self._current_command is not None and not self._current_command.done:
            return self._current_command.command_type.value
        else:
            return "IDLE"


    def run(self, current_time: float):

        if self._last_time is None:
            self._last_time = current_time
            return

        if not self._command_queue.empty() and (self._current_command is None or self._current_command.done):
            self._current_command = self._command_queue.get(block=False)

        if self._current_command is not None and not self._current_command.done and not self._current_command.submitted:
            if self._current_command.command_type == CommandType.ROTATE_COMMAND:
                self._target_angle = self._current_command.value[0]
                _delta_ang = self._target_angle - self._current_angle
                self._r_speed = self.rot_speed if _delta_ang >= 0 else -self.rot_speed
                self._delta_ang = abs(_delta_ang)
            elif self._current_command.command_type == CommandType.MOVEMENT_COMMAND:
                self._target_x_position = self._current_command.value[0]
                self._target_y_position = self._current_command.value[1]
                self._delta_vector = np.array([self._target_x_position, self._target_y_position]) - np.array(
                    [self._current_x_position, self._current_y_position])
            elif self._current_command.command_type == CommandType.PADDLE_COMMAND:
                self._target_paddle_position = self._current_command.value[0]
                _delta_paddle = self._target_paddle_position - self._current_paddle_position
                self._z_speed = self.paddle_speed if _delta_paddle >= 0 else  -self.paddle_speed
                self._delta_paddle = abs(_delta_paddle)

            elif self._current_command.command_type == CommandType.ALIGN_COMMAND:
                self._target_x_position = self._current_command.value[0]
                self._target_y_position = self._current_command.value[1]
                self._delta_vector = np.array([self._target_x_position, self._target_y_position]) - np.array(
                    [self._current_x_position, self._current_y_position])
                if self._delta_vector.any():
                    self._target_angle = np.atan2(self._delta_vector[1], self._delta_vector[0])
                    self._x_speed = self.shift_speed * self._delta_vector[0] / np.linalg.norm(self._delta_vector)
                    self._y_speed = self.shift_speed * self._delta_vector[1] / np.linalg.norm(self._delta_vector)

                _delta_ang = self._target_angle - self._current_angle
                self._r_speed = self.rot_speed if _delta_ang >= 0 else -self.rot_speed
                self._delta_ang = abs(_delta_ang)


            self._current_command.submitted = True


        if self._current_command is not None and not self._current_command.done and self._current_command.submitted and not self._paused:

            if self._current_command.command_type == CommandType.MOVEMENT_COMMAND:
                # where next iteration will be?
                next_x_position = self._current_x_position + self._x_speed * (current_time - self._last_time)
                next_y_position = self._current_y_position + self._y_speed * (current_time - self._last_time)

                next_delta_vector = np.array([self._target_x_position, self._target_y_position]) - np.array([next_x_position, next_y_position])

                if np.linalg.norm(next_delta_vector) < np.linalg.norm(self._delta_vector): # i'm getting closer
                    self._delta_vector = next_delta_vector
                    self._current_x_position = next_x_position
                    self._current_y_position = next_y_position
                else:   # i'm getting far, i will pass the target
                    self._current_x_position = self._target_x_position
                    self._current_y_position = self._target_y_position
                    self._current_command.done = True

            elif self._current_command.command_type == CommandType.ROTATE_COMMAND or self._current_command.command_type == CommandType.ALIGN_COMMAND:
                next_angle = self._current_angle + self._r_speed * (current_time - self._last_time)
                next_delta = abs(self._target_angle - next_angle)

                if next_delta < self._delta_ang:
                    self._current_angle = next_angle
                    self._delta_ang = next_delta
                else:
                    self._current_angle = self._target_angle
                    self._current_command.done = True
            elif self._current_command.command_type == CommandType.PADDLE_COMMAND:
                next_paddle = self._current_paddle_position + self._z_speed * (current_time - self._last_time)
                next_delta = abs(self._target_paddle_position - next_paddle)

                if next_delta < self._delta_paddle:
                    self._current_paddle_position = next_paddle
                    self._delta_paddle = next_delta
                else:
                    self._current_paddle_position = self._target_paddle_position
                    self._current_command.done = True

        self._last_time = current_time




if __name__ == '__main__':
    from matplotlib import pyplot as plt
    rob = Robot()

    t = []
    x_pos = []
    y_pos = []
    z_pos = []
    r_pos = []

    current_time = 0.0

    rob.move_to(10,10)
    rob.move_to(10,0)
    rob.move_to(0,0)
    rob.rotate(0)
    rob.move_paddle(10)

    rob.run(current_time)

    while rob.is_busy() and current_time < 20.0:
        current_time += 0.05
        rob.run(current_time)
        x,y,r = rob.get_robot_theta()
        z = rob.get_paddle_theta()

        x_pos.append(x)
        y_pos.append(y)
        z_pos.append(z)
        r_pos.append(r)

        t.append(current_time)


    fig1, (ax1,ax2,ax3) = plt.subplots(3)
    ax1.plot(t, x_pos, label='x_position')
    ax1.plot(t, y_pos, label='y_position')
    ax2.plot(t, z_pos, label='z_position')
    ax3.plot(t, np.array(r_pos)*180/np.pi, label='r_position')

    fig2, ax = plt.subplots()

    ax.plot(x_pos, y_pos, marker='o', linestyle='-')

    plt.legend()
    plt.show()







