import math
import numpy as np

class Car:
    def __init__(self,car_type, time_step=0.1):
        self.type = car_type        # 0: ego vehicle; 1: surrounding
        self.car_width = 1.0
        self.car_legnth = 4.7
        self.pos_x = 0
        self.pos_y = 0
        self.x_dot = 0
        self.y_dot = 0
        self.v_x = 0
        self.v_y = 0
        self.a_x = 0
        self.a_y = 0
        self.phi = 0
        self.omega = 0
        self.traj_x = []
        self.traj_y = []
        self.traj_phi = []
        self.center1_pos = []
        self.center2_pos = []
        self.dt = time_step
        self.r = 1.2
        self.kf = -128916           # cornering stiffness of the front wheels
        self.kr = -85944            # cornering stiffness of the rear wheels
        self.lf = 1.06              # distance from COM to the front axle
        self.lr = 1.85              # distance from COM to rear axle
        self.m = 1412               # mass of the vehicle
        self.Iz = 1536.7            # polar moment of inertia
        self.Lk = self.lf * self.kf - self.lr * self.kr

    def step(self, control_input):
        pos_x = self.pos_x + self.dt * (self.v_x * np.cos(self.phi) - self.v_y * np.sin(self.phi))
        pos_y = self.pos_y + self.dt * (self.v_y * np.cos(self.phi) + self.v_x * np.sin(self.phi))
        phi = self.phi + self.dt * self.omega
        v_x = self.v_x + self.dt * control_input[0]
        v_y = (self.m * self.v_x * self.v_y + self.dt * (self.Lk * self.omega - self.kf * control_input[1] * self.v_x - self.m * self.v_x * self.v_x * self.omega)) / (self.m * self.v_x - self.dt * (self.kr + self.kf))
        omega = (self.Iz * self.v_x * self.omega + self.dt * (self.Lk * self.v_y - self.lf * self.kf * control_input[1] * self.v_x)) / (self.Iz * self.v_x - self.dt * (self.lf * self.lf * self.kf + self.lr * self.lr * self.kr))
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.phi = phi
        self.v_x = v_x
        self.v_y = v_y
        self.x_dot = self.v_x * math.cos(self.phi) - self.v_y * math.sin(self.phi)
        self.y_dot = self.v_x * math.sin(self.phi) + self.v_y * math.cos(self.phi)
        self.omega = omega
        self.a_x = control_input[0] * np.cos(self.phi)
        self.a_y = control_input[0] * np.sin(self.phi)

    def step_test(self, u_ax, u_ay):
        self.pos_x += self.v_x * self.dt
        self.pos_y += self.v_y * self.dt
        self.v_x += u_ax * self.dt
        self.v_y += u_ay * self.dt
        # self.phi += self.omega * self.dt

    def updateTrajectoryPhi(self):
        self.traj_phi[0] = self.phi
        for i in range(len(self.traj_x) - 1):
            self.traj_phi[i + 1] = math.atan2((self.traj_y[i+1] - self.traj_y[i])/self.dt, (self.traj_x[i+1]-self.traj_x[i])/self.dt)

    def updateCenterPos(self):
        for i in range(len(self.traj_x)):
            self.center1_pos[i][0] = self.traj_x[i] + self.r * math.sin(self.traj_phi[i])
            self.center2_pos[i][0] = self.traj_x[i] - self.r * math.sin(self.traj_phi[i])
            self.center1_pos[i][1] = self.traj_y[i] + self.r * math.cos(self.traj_phi[i])
            self.center2_pos[i][1] = self.traj_y[i] - self.r * math.cos(self.traj_phi[i])

class Lane:
    def __init__(self, num_of_lane, line_width):
        self.num_of_lane = num_of_lane
        self.line_width = line_width
    
    def getTotalWidth(self):
        return self.num_of_lane * self.line_width

class Env:
    def __init__(self, ego_vehicle, surrounding_vehicle_list):
        self.ego_vehicle = ego_vehicle
        self.surrounding_vehicle_list = surrounding_vehicle_list

    def collisionCheckForAllCar(self):
        for car in self.surrounding_vehicle_list:
            for i in range(len(self.ego_vehicle.traj_x)):
                if self.collisionCheck(self.ego_vehicle, car, i):
                    return True         # collision detected
        return False                    # no collision

    def collisionCheck(self, car1, car2, index):
        if self.eulerDistance(car1.center1_pos[index], car2.center1_pos[index]) <= self.r or self.eulerDistance(car1.center1_pos[index], car2.center2_pos[index]) <= self.r or\
            self.eulerDistance(car1.center2_pos[index], car2.center1_pos[index]) <= self.r or self.eulerDistance(car1.center2_pos[index], car2.center2_pos[index]) <= self.r:
            return True
        else:
            return False

    def eulerDistance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 +(point1[1] - point2[1]) ** 2)