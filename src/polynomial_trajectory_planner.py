import numpy as np
import random

class Sampler:
    def __init__(self, x_up_limit, x_low_limit, y_up_limit, y_low_limit, v_x_limit, v_y_limit, a_low_limit, a_up_limit, sample_time):
        self.x_up_limit = x_up_limit
        self.x_low_limit = x_low_limit
        self.y_up_limit = y_up_limit
        self.y_low_limit = y_low_limit
        self.v_x_limit = v_x_limit
        self.v_y_limit = v_y_limit
        self.a_low_limit = a_low_limit
        self.a_up_limit = a_up_limit
        self.sample_time = sample_time

    def sample_longitude(self, p_init, v_init, flag = True):
        # flag = True: end velocity equal initial velocity; flag = False: end velocity has to greater than initial velocity
        if flag:
            end_velocity = v_init
        else:
            end_velocity = (random.random() * (self.a_up_limit - self.a_low_limit) + self.a_low_limit)* self.sample_time * 0.5 + v_init         # since a is not only in longitude direction, multiply it with 0.5 to ensure feasibility
        end_position = random.random() * v_init * self.sample_time + p_init
        return end_position, end_velocity
        
class Polynomial:
    def __init__(self, end_time):
        self.motionTime = 0
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.a5 = 0             # Quintic Polynomials
        self.T = end_time
        self.trajectory = []
        self.fitness = 0        # 

    def computeParam(self, start_pos, start_vel, start_acc, end_pos, end_vel, end_acc):
        A = np.matrix([1,       0,      0,              0,                  0,                  0],
                      [0,       1,      0,              0,                  0,                  0],
                      [0,       0,      2,              0,                  0,                  0],
                      [1,       self.T, self.T ** 2,    self.T ** 3,        self.T ** 4,        self.T ** 5],
                      [0,       1,      2 * self.T,     3 * self.T ** 2,    4 * self.T ** 3,    5 * self.T ** 4],
                      [0,       0,      2,              6 * self.T,         12 * self.T ** 2,   20 * self.T ** 3])
        b = np.matrix([start_pos], [start_vel], [start_acc], [end_pos], [end_vel], [end_acc])
        x = np.linalg.inv(A) * b
        return x

    def getTrajectory(self, start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, sample_point):
        param = self.computeParam(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc)
        self.a0 = param[0]
        self.a1 = param[1]
        self.a2 = param[2]
        self.a3 = param[3]
        self.a4 = param[4]
        self.a5 = param[5]
        self.trajectory = []
        t_current = 0
        for i in range(sample_point):
            self.trajectory.append(self.a0 + self.a1 * t_current + self.a2 * t_current ** 2 + self.a3 * t_current ** 3 + self.a4 * t_current ** 4 + self.a5 * t_current ** 5)
            t_current += 1 / (sample_point - 1)