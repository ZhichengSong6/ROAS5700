import numpy as np
import math 

class DynamicsModel:
    def __init__(self,px,py,phi,vx,vy,omega):
        # parameter
        self.kf = -128916           # cornering stiffness of the front wheels
        self.kr = -85944            # cornering stiffness of the rear wheels
        self.lf = 1.06              # distance from COM to the front axle
        self.lr = 1.85              # distance from COM to rear axle
        self.m = 1412               # mass of the vehicle
        self.Iz = 1536.7            # polar moment of inertia
        self.Ts = 0.1               # time step
        self.Lk = self.lf * self.kf - self.lr * self.kr
        # state
        self.px = px                # X-coordinate of the center point of the vehicle
        self.py = py                # Y-coordinate of the center point of the vehicle
        self.phi = phi              # heading angle
        self.vx = vx                # longitudinal velocity
        self.vy = vy                # lateral velocity
        self.omega = omega          # yaw rate

    def nonlinear_discrete_model_step_forward(self,control_input):
        px = self.px + self.Ts*(self.vx * math.cos(self.phi) - self.vy * math.sin(self.phi))
        py = self.py + self.Ts*(self.vy * math.cos(self.phi) + self.vx * math.sin(self.phi))
        phi = self.phi + self.Ts * self.omega
        vx = self.vx + self.Ts * control_input[1]
        vy = (self.m * self.vx * self.vy + self.Ts * (self.Lk * self.omega - self.kf * control_input[0] * self.vx - self.m * self.vx * self.vx * self.omega)) / (self.m * self.vx - self.Ts * (self.kr + self.kf))
        omega = (self.Iz * self.vx * self.omega + self.Ts * (self.Lk * self.vy - self.lf * self.kf * control_input[1] * self.vx)) / (self.Iz * self.vx - self.Ts * (self.lf * self.lf * self.kf + self.lr * self.lr * self.kr))
        self.py = py
        self.phi = phi
        self.vx = vx
        self.vy = vy
        self.omega = omega