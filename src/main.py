from polynomial_trajectory_planner import Polynomial
from polynomial_trajectory_planner import Sampler
from environment import Car
from environment import Lane
from environment import Env
import random

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
ego_car.pos_x = 2
ego_car.pos_y = 2

# initial Env
env = Env(ego_car,surrounding_car_list)

# initialize poly for surrounding car and ego car
poly_for_surrounding_car_list = [] 
for _ in range(num_of_surrounding_car):
    poly_for_surrounding_car_list.append(Polynomial(3))  # sample time T = 3

poly_for_ego_car = Polynomial(3)

# initialize lane and sampler
lane = Lane(2, 4)
sampler = Sampler(300, 1.5 * ego_car.r, lane.line_width * lane.num_of_lane - ego_car.r, ego_car.r, 25, 10, -3, 1.5, poly_for_ego_car.motionTime)


while True:
    # generate trajectory for surrounding car
    for i in range(num_of_surrounding_car):
        poly_for_surrounding_car_list[i].getTrajectory(surrounding_car_list[i].pos_x, surrounding_car_list[i].ego_car.v_x, 0,\
                                                        surrounding_car_list[i].pos_x + surrounding_car_list[i].ego_car.v_x * poly_for_surrounding_car_list[i].T, surrounding_car_list[i].ego_car.v_x, 0, 15)
    # poly for recording best trajectory get by the sampler
    poly_sample = Polynomial(3)
    # sample for 20 points for each time
    for i in range(20):
        end_pos_x, end_vel_x = sampler.sample_longitude(ego_car.pos_x, ego_car.v_x)
        poly_sample.getTrajectory(ego_car.v_x, ego_car.v_y, ego_car.a_x, end_pos_x, end_vel_x, 0, 15)
        if not env.collisionCheckForAllCar():
            poly_sample.fitness = -1
        else:
            
