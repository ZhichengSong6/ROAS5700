import math

class Car:
    def __init__(self,car_type):
        self.type = car_type        # 0: ego vehicle; 1: surrounding
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
        self.center1_pos = []
        self.center2_pos = []
        self.r = 1.2

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
            for i in range(len(self.ego_vehicle.traj)):
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