from math import cos, sin, pi
from Integrator import Integrator

class Odometry:
    def __init__(self, r: float, B: float, T: float):
        self.r = r
        self.B = B
        self.T = T
        self.x_integrator = Integrator(0, T)
        self.y_integrator = Integrator(0, T)
        self.theta_integrator = Integrator(0, T)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def get_speed(self, wl: float, wr: float) -> tuple:
        v = (wr + wl) * self.r / 2
        w = (wr - wl) * self.r / self.B
        dx = v * cos(self.theta)
        dy = v * sin(self.theta)
        dth = w 
        return (dx, dy, dth)

    def update(self, wl: float, wr: float) -> tuple:
        dx, dy, dth = self.get_speed(wl, wr)

        self.x = self.x_integrator.update(dx)
        self.y = self.y_integrator.update(dy)
        self.theta = self.theta_integrator.update(dth)

        return (self.x, self.y, self.theta)
        