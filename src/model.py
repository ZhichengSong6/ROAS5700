import numpy as np
import math 
import casadi as ca

class DynamicsModel:
    def __init__(self, N, Q, R, x_ref, iter):
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
        self.N = N                               # number of control intervals
        self.Q = Q
        self.R = R
        self.x_ref_symb = ca.MX.sym('x_ref', 6)
        self.x_ref = x_ref[iter : iter + self.N].ravel()
        self.x_ref_forward = x_ref[iter: iter + self.N]

    def getModelEquation(self):
        return ca.vertcat(
            self.px + self.Ts * (self.vx * np.cos(self.phi) - self.vy * np.sin(self.phi)),
            self.py + self.Ts * (self.vy * np.cos(self.phi) + self.vx * np.sin(self.phi)),
            self.phi + self.Ts * self.omega,
            self.vx + self.Ts * self.a,
            (self.m * self.vx * self.vy + self.Ts * (self.Lk * self.omega - self.kf * self.delta * self.vx - self.m * self.vx * self.vx * self.omega)) / (self.m * self.vx - self.Ts * (self.kr + self.kf)),
            (self.Iz * self.vx * self.omega + self.Ts * (self.Lk * self.vy - self.lf * self.kf * self.delta * self.vx)) / (self.Iz * self.vx - self.Ts * (self.lf * self.lf * self.kf + self.lr * self.lr * self.kr))
        )
    
    def getObjectiveFunction(self):
        return (self.x.T - self.x_ref_symb.T) @ self.Q @ (self.x - self.x_ref_symb) + self.u.T @ self.R @ self.u
    
    def solve(self, initial_state):
        # formulate MPC Problem
        f = ca.Function('FD',[self.x, self.u, self.x_ref_symb],[self.getModelEquation(), self.getObjectiveFunction()], ['state', 'control_input', 'ref_state'],['next_step_state', 'cost'])
        Xk = ca.MX.sym('X_0',6)
        w = [Xk]
        w0 = [0,0,0,0,0,0]
        lbw = [initial_state[0], initial_state[1], initial_state[2], initial_state[3], initial_state[4], initial_state[5]]
        ubw = [initial_state[0], initial_state[1], initial_state[2], initial_state[3], initial_state[4], initial_state[5]]
        p = []
        g = []
        lbg = []
        ubg = []
        J = 0

        # Xk = ca.MX(initial_state)
        for k in range(self.N):
            Uk = ca.MX.sym('U_' + str(k), 2)
            w += [Uk]
            lbw += [-3, -0.6]
            ubw += [1.5, 0.6]
            w0 += [0,0]
            pk = ca.MX.sym('p_' + str(k), 6)
            p += [pk]
        
            # Integrate till the end of the interval
            fk = f(state=Xk, control_input=Uk, ref_state=pk)
            Xnext = fk['next_step_state']
            J = J + fk['cost']
            Xk = ca.MX.sym('X_' + str(k + 1), 6)
            w += [Xk]
            lbw += 6 * [-np.inf]
            ubw += 6 * [np.inf]
            w0 += [0,0,0,0,0,0]

            g += [Xnext - Xk]
            lbg += 6 * [0]
            ubg += 6 * [0]

        # create NLP solver
        nlp = {'f': J, 'x': ca.vertcat(*w), 'g': ca.vertcat(*g), 'p': ca.vertcat(*p)}
        solver = ca.nlpsol('solver', 'ipopt', nlp)
        # solve the NLP
        sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg, p=self.x_ref)

        w_opt = sol['x']
        u_opt = []
        x1_opt = []
        x2_opt = []
        x3_opt = []
        x4_opt = []
        x5_opt = []
        x6_opt = []
        for i in range(self.N):
            u_opt.append(float(w_opt[8*i+6]))
            u_opt.append(float(w_opt[8*i+7]))
            x1_opt.append(float(w_opt[8*i+0]))
            x2_opt.append(float(w_opt[8*i+1]))
            x3_opt.append(float(w_opt[8*i+2]))
            x4_opt.append(float(w_opt[8*i+3]))
            x5_opt.append(float(w_opt[8*i+4]))
            x6_opt.append(float(w_opt[8*i+5]))
        # print("u_opt: ", u_opt[0:2])
        # print("w_opt: ", w_opt)

        # x1_opt = ca.vcat([r[0] for r in x_opt])
        # x2_opt = ca.vcat([r[1] for r in x_opt])
        # x3_opt = ca.vcat([r[2] for r in x_opt])
        # x4_opt = ca.vcat([r[3] for r in x_opt])
        # x5_opt = ca.vcat([r[4] for r in x_opt])
        # x6_opt = ca.vcat([r[5] for r in x_opt])
        

        # print('x_state',x1_opt)
        # print("x_ref",self.x_ref_forward[:,0])
        # print('y_state',x2_opt)
        # print("y_ref",self.x_ref_forward[:,1])
        # print('phi_state',x3_opt)
        # print("phi_ref",self.x_ref_forward[:,2])
        # print('vx_state',x4_opt)
        # print("vx_ref",self.x_ref_forward[:,3])
        # print('vy_state',x5_opt)
        # print("vy_ref",self.x_ref_forward[:,4])
        # print('omega_state',x6_opt)
        # print("omega_ref",self.x_ref_forward[:,5])

        # # tgrid = [self.T/self.N*k for k in range(self.N+1)]
        # import matplotlib.pyplot as plt
        # plt.figure(1)
        # plt.clf()
        # plt.plot(x1_opt, x2_opt, '--')
        
        # plt.plot(self.x_ref_forward[:,0], self.x_ref_forward[:,1], '-')
        # # plt.plot(tgrid, x2_opt, '-')
        # # plt.step(tgrid, ca.vertcat(u_opt), '-.')
        # # plt.xlabel('t')
        # # plt.legend(['x1','x2','u'])
        # plt.grid()
        # plt.show()

        return u_opt[0:2]