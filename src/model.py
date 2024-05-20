import numpy as np
import math 
import casadi as ca

class DynamicsModel:
    def __init__(self,T, N, Q, R, x_ref):
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
        self.px = ca.MX.sym('px')                # X-coordinate of the center point of the vehicle
        self.py = ca.MX.sym('py')                # Y-coordinate of the center point of the vehicle
        self.phi = ca.MX.sym('phi')              # heading angle
        self.vx = ca.MX.sym('vx')                # longitudinal velocity
        self.vy = ca.MX.sym('vy')                # lateral velocity
        self.omega = ca.MX.sym('omega')          # yaw rate
        self.x = ca.vertcat(self.px, self.py, self.phi, self.vx, self.vy, self.omega)
        # control
        self.a = ca.MX.sym('a')
        self.delta = ca.MX.sym('delta')
        self.u = ca.vertcat(self.a, self.delta)
        self.T = T                               # Time horizon
        self.N = N                               # number of control intervals
        self.Q = Q
        self.R = R
        self.x_ref = x_ref
        self.x_ref_curr = x_ref[0]

    def getModelEquation(self):
        return ca.vertcat(
            self.px + self.Ts*(self.vx * np.cos(self.phi) - self.vy * np.sin(self.phi)),
            self.py + self.Ts*(self.vy * np.cos(self.phi) + self.vx * np.sin(self.phi)),
            self.phi + self.Ts * self.omega,
            self.vx + self.Ts * self.a,
            (self.m * self.vx * self.vy + self.Ts * (self.Lk * self.omega - self.kf * self.delta * self.vx - self.m * self.vx * self.vx * self.omega)) / (self.m * self.vx - self.Ts * (self.kr + self.kf)),
            (self.Iz * self.vx * self.omega + self.Ts * (self.Lk * self.vy - self.lf * self.kf * self.delta * self.vx)) / (self.Iz * self.vx - self.Ts * (self.lf * self.lf * self.kf + self.lr * self.lr * self.kr))
        )
    
    def getObjectiveFunction(self):
        return (self.x.T - self.x_ref_curr.T)* self.Q * (self.x - self.x_ref_curr) + self.u.T * self.R * self.u
    
    def solve(self, initial_state):
        # formulate MPC Problem
        f = ca.Function("FD",[self.x, self.u],[self.getModelEquation(), self.getObjectiveFunction], ['state', 'control_input'],["next_step_state", "cost"])
        Xk = ca.MX(initial_state)
        w = []
        w0 = []
        lbw = []
        ubw = []
        J = 0
        # fk = f(state=[0,0,0,0,0,0],control_input=[1,0])
        # print(fk['next_step_state'])

        for k in range(self.N):
            Uk = ca.MX.sym('U_' + str(k))
            w += [Uk]
            lbw += [-3, -0.6]
            ubw += [1.5, 0.6]
            w0 += [0,0]
        
            # Integrate till the end of the interval
            fk = f(state=Xk,control_input=Uk)
            Xk = fk['next_step_state']
            J = J + fk['cost']

            self.x_ref_curr = self.x_ref[k+1]
        
        # create NLP solver
        nlp = {'f': J, 'x': ca.vertcat(*w), 'p': ca.vertcat(self.x), 'g': ca.vertcat()}
        solver = ca.nlpsol('solver', 'ipopt', nlp)
        # solve the NLP
        sol = solver(x0=w0, lbx=lbw, ubx=ubw, p=initial_state)