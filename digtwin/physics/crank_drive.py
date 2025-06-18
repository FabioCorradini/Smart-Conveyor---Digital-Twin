import numpy as np
import time


class CrankDrive:
    """
                       *B
      *O             /  (ψ)
    (θ)\\         /
        \\ (α) /
         *A
    """

    def __init__(self):
        #settings
        self.max_gate_angle = np.pi/4
        # cinematic
        self.a = np.array([[+34.3], [-58.1]]) # crank, rod joint
        self.b = np.array([[+197.2], [+59.0]]) # piston, frame joint
        self.ab = (self.b - self.a).flatten() # rod vector
        self.psi = np.arctan2(self.ab[0], self.ab[1])  # rod angle
        self.theta_0 = np.arctan2(self.a[0], -self.a[1]) # crank angle
        self.theta_max = self.theta_0 + self.max_gate_angle

        #dinamic
        self._acc_max =  120*np.pi/180 # rad/s^2  corresponding to 5 bar
        self._acc_min = 40*np.pi/180  # rad/s^2  corresponding to 1 bar
        self._a_acc = 0.0
        self._movement_enabled = True
        self._old_time = None

        self._dec_area = 11.25*np.pi/180 # rad starting of dynamic brake

        self.attr_k = 4 # 1/s coefficient of aerodinamic friction
        self.attr_k2 = 6 # 1/s coefficient of aerodinamic friction in off area

        self.moving_forward = True

        self._current_pos = .0
        self._current_speed = .0
        self._current_acc = .0

        self._old_speed = .0
        self._old_acc = .0
        self._old_pos = .0

        self.pressure = 5e5



    @property
    def pressure(self) -> float:
        return self._pressure

    @pressure.setter
    def pressure(self, pressure: float):
        self._pressure = pressure
        if self._pressure < 1e5:
            self._movement_enabled = False
        else:
            self._movement_enabled = True
            self._a_acc = np.interp(self._pressure, [1e5, 5e5], [self._acc_min, self._acc_max])

    @property
    def theta(self):
        return self._current_pos


    def run(self, current_time: float):

        if self._old_time is None:
            self._old_time = current_time
            return

        if self._movement_enabled:
            if self.moving_forward:
                if self._current_pos < self.max_gate_angle - self._dec_area:
                    # not in deceleration area
                    self._current_acc = self._a_acc - self._current_speed * self.attr_k
                else:
                    self._current_acc = self._a_acc - self._current_speed * self.attr_k2
            else:
                if self._current_pos > self._dec_area:
                    self._current_acc =  - self._a_acc - self._current_speed * self.attr_k
                else:
                    self._current_acc = -self._a_acc - self._current_speed * self.attr_k2

            if self._current_pos <= 0.0 and not self.moving_forward:
                self._current_speed = 0.0
                self._current_pos = 0.0
            elif self._current_pos >= self.max_gate_angle and self.moving_forward:
                self._current_speed = 0.0
                self._current_pos = self.max_gate_angle
            else:
                self._current_speed = self._old_speed + (self._old_acc + self._current_acc)*(current_time - self._old_time)/2
                self._current_pos = self._old_pos + (self._old_speed + self._current_speed)*(current_time - self._old_time)/2

            self._old_time = current_time
            self._old_pos = self._current_pos
            self._old_speed = self._current_speed
            self._old_acc = self._current_acc


    def get_d_psi_d_alpha(self, d_theta: float) -> tuple[float, float]:
        c, s = np.cos(d_theta), np.sin(d_theta)
        R = np.array(((c, -s), (s, c)))
        A_1 = np.matmul(R, self.a).flatten()
        V_1 = self.b.flatten() - A_1
        psi_1 = np.atan2(V_1[0], V_1[1])
        return float(psi_1 - self.psi) , float(d_theta - self.psi + psi_1)


if __name__ == '__main__':
    from matplotlib import pyplot as plt

    cd = CrankDrive()
    p = []
    v = []
    a = []
    t = []
    current_time = 0

    cd.run(current_time)

    # cd.pressure = 1e5


    while cd.theta < cd.max_gate_angle/2:
        current_time += 0.01
        cd.run(current_time)
        p.append(cd.theta * 180 / np.pi)
        v.append(cd._current_speed * 180/np.pi)
        a.append(cd._current_acc * 180/np.pi)
        t.append(current_time)

    cd.moving_forward = False

    while cd.theta > 0:
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





