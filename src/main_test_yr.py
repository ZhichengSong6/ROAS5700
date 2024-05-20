from polynomial_trajectory_planner import Polynomial
from polynomial_trajectory_planner import Sampler
from environment import Car
from environment import Lane
import random
import numpy as np
import matplotlib.pyplot as plt

# initialize cars

num_of_surrounding_car = 1                          

surrounding_car_list = []
for _ in range(num_of_surrounding_car):
    car = Car(1)
    car.v_x = 8
    car.pos_y = random.choice([2,4])
    car.pos_x = random.randint(15,30)
    surrounding_car_list.append(car)

# locate surrounding_car

# specify ego car state
ego_car = Car(0)
ego_car.v_x = 6
ego_car.pos_x = 0
ego_car.pos_y = -4.0/2.0

# initialize poly for surrounding car and ego car
poly_for_surrounding_car_list = [] 
for _ in range(num_of_surrounding_car):
    poly_for_surrounding_car_list.append(Polynomial(3))  # sample time T = 3

poly_for_ego_car = Polynomial(3)

# initialize lane and sampler
lane = Lane(2, 4)
sampler = Sampler(300, 1.5 * ego_car.r, lane.line_width * lane.num_of_lane - ego_car.r, ego_car.r, 10, 25, -3, 1.5)


while True:
    # sample for 20 trajectories for each time
    kNumSample = 4
    kSamplePoints = 10
    lc_times = []
    kMinLcTime = 3.0
    kMaxLcTime = 10.0
    for i in range(kNumSample):
        lc_time = kMinLcTime + (kMaxLcTime - kMinLcTime) / kNumSample * i
        lc_times.append(lc_time)
    for lc_time in lc_times:

        end_pos_x, end_vel_x = sampler.sample_longitude(ego_car.pos_x, ego_car.v_x, lc_time)
        print("lc_time: ", lc_time, ", end_x: ", end_pos_x)
        init_state = np.array([0,        -lane.line_width / 2, 6, 0, 0, 0])  # x,y; vx,vy; ax,ay
        end_state = np.array([end_pos_x, lane.line_width / 2,  end_vel_x, 0, 0, 0])
        poly_for_ego_car = Polynomial(lc_time)
        ego_plan_traj = poly_for_ego_car.getTrajectory(init_state, end_state)
        poly_for_ego_car.plotPath(ego_plan_traj, init_state, end_state, lane.line_width, 35, car.car_width, car.car_legnth)

    break
