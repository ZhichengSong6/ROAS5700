import numpy as np
import random
import matplotlib.pyplot as plt

class Sampler:
    def __init__(self, x_up_limit, x_low_limit, y_up_limit, y_low_limit, v_x_limit, v_y_limit, a_low_limit, a_up_limit):
        self.x_up_limit = x_up_limit
        self.x_low_limit = x_low_limit
        self.y_up_limit = y_up_limit
        self.y_low_limit = y_low_limit
        self.v_x_limit = v_x_limit
        self.v_y_limit = v_y_limit
        self.a_low_limit = a_low_limit
        self.a_up_limit = a_up_limit
        # self.sample_time = sample_time

    def sample_longitude(self, p_init, v_init, sample_time, flag = True):
        # flag = True: end velocity equal initial velocity; flag = False: end velocity has to greater than initial velocity
        if flag:
            end_velocity = v_init
            end_position = v_init * sample_time + p_init
        else:
            end_velocity = (random.random() * (self.a_up_limit - self.a_low_limit) + self.a_low_limit)* sample_time * 0.5 + v_init         # since a is not only in longitude direction, multiply it with 0.5 to ensure feasibility
            end_position = random.random() * v_init * sample_time + p_init
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

        self.dt = 0.1
        self.t0 = 0
        self.t1 = self.T
        self.A = None
        self.B = None

    def computeParam(self, init_state, end_state):
        # init state and end state
        t0 = self.t0
        t1 = self.t1

        # concatenate
        X = np.concatenate((np.array([init_state[i] for i in range(6) if i % 2 == 0]),
                            np.array([end_state[i] for i in range(6) if i % 2 == 0])))
        Y = np.concatenate((np.array([init_state[i] for i in range(6) if i % 2 != 0]),
                            np.array([end_state[i] for i in range(6) if i % 2 != 0])))

        # 矩阵T表示
        T = np.matrix([
            [t0 ** 5, t0 ** 4, t0 ** 3, t0 ** 2, t0, 1],
            [5 * t0 ** 4, 4 * t0 ** 3, 3 * t0 ** 2, 2 * t0, 1, 0],
            [20 * t0 ** 3, 12 * t0 ** 2, 6 * t0, 1, 0, 0],
            [t1 ** 5, t1 ** 4, t1 ** 3, t1 ** 2, t1, 1],
            [5 * t1 ** 4, 4 * t1 ** 3, 3 * t1 ** 2, 2 * t1, 1, 0],
            [20 * t1 ** 3, 12 * t1 ** 2, 6 * t1, 1, 0, 0]
        ])

        # # Solution 1
        # self.A=np.linalg.pinv(T)@X
        # self.B=np.linalg.pinv(T)@Y.T
        # self.A=self.A.T
        # self.B=self.B.T

        # # Solution 2
        self.A = np.linalg.solve(T, X)
        self.B = np.linalg.solve(T, Y)

    def getTrajectory(self, init_state, end_state):


        # init and end state
        self.computeParam(init_state, end_state)

        # get timeline to obtain discrete trajectory
        t = np.transpose((np.arange(self.t0, self.t1 + self.dt, self.dt)))
        path = np.zeros((len(t), 4))  # x,y,vx,vy

        for i in range(len(t)):
            # x
            path[i, 0] = np.array([t[i] ** 5, t[i] ** 4, t[i] ** 3, t[i] ** 2, t[i], 1]) @ self.A
            # y
            path[i, 1] = np.array([t[i] ** 5, t[i] ** 4, t[i] ** 3, t[i] ** 2, t[i], 1]) @ self.B
            # vx
            path[i, 2] = np.array([5 * t[i] ** 4, 4 * t[i] ** 3, 3 * t[i] ** 2, 2 * t[i], 1, 0]) @ self.A
            # vy
            path[i, 3] = np.array([5 * t[i] ** 4, 4 * t[i] ** 3, 3 * t[i] ** 2, 2 * t[i], 1, 0]) @ self.B

        return path

    def plotPath(self, path, init_state, end_state, lane_width, line_length, car_width, car_length):
        # params
        d = lane_width
        len_line = line_length
        W = car_width
        L = car_length

        # time line
        t = np.transpose((np.arange(self.t0, self.t1 + self.dt, self.dt)))

        ## 画场景示意图
        plt.figure(1)
        # 画灰色路面图
        GreyZone = np.array([[- 5, - d - 0.5], [- 5, d + 0.5], [len_line, d + 0.5], [len_line, - d - 0.5]])
        plt.fill(GreyZone[:, 0], GreyZone[:, 1], np.array([0.5, 0.5, 0.5]))

        # 画小车
        plt.fill(np.array([init_state[0], init_state[0], init_state[0] + L, init_state[0] + L]),
                 np.array([- d / 2 - W / 2, - d / 2 + W / 2, - d / 2 + W / 2, - d / 2 - W / 2]), 'b')
        plt.fill(np.array([end_state[0], end_state[0], end_state[0] - L, end_state[0] - L]),
                 np.array([ d / 2 - W / 2,  d / 2 + W / 2,  d / 2 + W / 2,  d / 2 - W / 2]), 'y')
        # 画分界线
        plt.plot(np.array([- 5, len_line]), np.array([0, 0]), 'w--')

        plt.plot(np.array([- 5, len_line]), np.array([d, d]), 'w')

        plt.plot(np.array([- 5, len_line]), np.array([- d, - d]), 'w')

        # 设置坐标轴显示范围
        plt.axis('equal')

        plt.title("scene")

        # 画换道轨迹
        plt.plot(path[:, 0], path[:, 1], 'r--')
        ## 分析速度

        # # 纵向速度
        # plt.figure(3)
        # plt.plot(t, path[:, 2], 'k')
        # plt.xlabel('time/s ')
        # plt.ylabel('m/s ')
        # plt.title("longi vel")
        #
        # # 横向速度
        # plt.figure(2)
        # plt.plot(t, path[:, 3], 'k')
        # plt.xlabel('time/s ')
        # plt.ylabel('m/s ')
        # plt.title("lat vel")

        plt.show()