import numpy as np
import time


class Motor:

    def __init__(self):
        #dinamic
        self._a_acc = 2*np.pi # rad / s^2

        self.moving_forward = True
        self.running = True
        self.movement_speed = 2*np.pi # rad / s

        self._current_pos = .0
        self._current_speed = .0
        self._current_acc = .0

        self._old_speed = .0
        self._old_acc = .0
        self._old_pos = .0
        self._old_time = None


    @property
    def theta(self):
        return self._current_pos

    def run(self, current_time: float):

        if self._old_time is None:
            self._old_time = current_time
            return

        if self.moving_forward:
            if self.running:
                self._current_acc = self._a_acc
            else:
                self._current_acc = - self._a_acc


            if self.running and self._current_speed >= self.movement_speed :
                self._current_speed = self.movement_speed
                self._current_acc = 0.0
            elif not self.running and self._current_speed <= 0:
                self._current_acc = 0.0
                self._current_speed = 0.0
            else:
                self._current_speed = self._old_speed + (self._old_acc + self._current_acc)*(current_time - self._old_time)/2

        else:
            if self.running:
                self._current_acc = - self._a_acc
            else:
                self._current_acc = self._a_acc

            if self.running and self._current_speed <= - self.movement_speed :
                self._current_speed = - self.movement_speed
                self._current_acc = 0.0
            elif not self.running and self._current_speed >= 0:
                self._current_speed = 0.0
                self._current_acc = 0.0
            else:
                self._current_speed = self._old_speed + (self._old_acc + self._current_acc)*(current_time - self._old_time)/2

        self._current_pos = self._old_pos + (self._old_speed + self._current_speed)*(current_time - self._old_time)/2

        self._old_time = current_time
        self._old_pos = self._current_pos
        self._old_speed = self._current_speed
        self._old_acc = self._current_acc


if __name__ == '__main__':
    from matplotlib import pyplot as plt

    cd = Motor()
    p = []
    v = []
    a = []
    t = []
    current_time = 0

    cd.run(current_time)

    cd.running = True

    while current_time < 2.00:
        current_time += 0.01
        cd.run(current_time)
        p.append(cd.theta * 180 / np.pi)
        v.append(cd._current_speed * 180/np.pi)
        a.append(cd._current_acc * 180/np.pi)
        t.append(current_time)

    cd.moving_forward = False

    while current_time < 4.00:
        current_time += 0.01
        cd.run(current_time)
        p.append(cd.theta * 180 / np.pi)
        v.append(cd._current_speed * 180/np.pi)
        a.append(cd._current_acc * 180/np.pi)
        t.append(current_time)

    fig, (ax1,ax2,ax3) = plt.subplots(3)
    ax1.plot(t, p, label='position')
    ax2.plot(t, v, label='velocity')
    ax3.plot(t, a, label='acceleration')
    plt.legend()
    plt.show()





