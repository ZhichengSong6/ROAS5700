import math

class Car:
    def __init__(self,car_type, time_step=0.02):
        self.type = car_type        # 0: ego vehicle; 1: surrounding
        self.car_width = 1.0
        self.car_legnth = 4.7
        self.pos_x = 0
        self.pos_y = 0
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

    def step(self):
        self.pos_x += self.v_x * self.dt
        self.pos_y += self.v_y * self.dt
        self.v_x += self.a_x * self.dt
        self.v_y += self.a_y * self.dt
        self.phi += self.omega * self.dt 

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