from polynomial_trajectory_planner import Polynomial
from polynomial_trajectory_planner import Sampler
from environment import Car
from environment import Lane
from environment import Env
import random
from trajectory_decision import TrajectoryDecisionForLaneChanging
from trajectory_decision import TrajectoryDecisionForOvertaking
from utils import *
import math
from model import DynamicsModel

kSimTime = 15.0

# initialize cars
num_of_surrounding_car = 1
surrounding_car_list = []
for _ in range(num_of_surrounding_car):
    car = Car(1)
    car.x_dot = 5
    car.v_x  = 5
    # lane changing
    # car.pos_y = 2 #random.choice([2,4])
    # car.pos_x = 0 #random.randint(15,30)
    #overtaking
    car.pos_y = -2 #random.choice([2,4])
    car.pos_x = 20 #random.randint(15,30)
    surrounding_car_list.append(car)

# specify ego car state
ego_car = Car(0)
ego_car.x_dot = 6
ego_car.v_x  = 6
ego_car.pos_x = 2
ego_car.pos_y = -4.0/2.0

# initial Env
lane_width = 4.0
lane_num = 2
lane = Lane(lane_num, lane_width)
env = Env(ego_car, surrounding_car_list, lane, 0.1, kSimTime+10)

# initialize poly for surrounding car and ego car
poly_for_surrounding_car_list = []
for _ in range(num_of_surrounding_car):
    poly_for_surrounding_car_list.append(Polynomial(3))  # sample time T = 3

# initialize sampler
sampler = Sampler(300, 1.5 * ego_car.r, lane.line_width * lane.num_of_lane - ego_car.r, ego_car.r, 10, 25, -3, 1.5)


def prepareStateForControl(path):
        # note that vx vy from polynomial is not the same as vx vy from car

        control_traj = np.array(np.zeros((len(path), 6)))

        control_traj[0][5] = 0                                                                              # omega
        for i in range(len(ego_plan_traj)-1):
            control_traj[i][0] = path[i][0]                                                                 # x
            control_traj[i][1] = path[i][1]                                                                 # y
            control_traj[i][2] = math.atan2(path[i][3], path[i][2])                                         # phi
            control_traj[i][3] = path[i][2] * math.cos(control_traj[i][2]) + path[i][3] * math.sin(control_traj[i][2])                                     # vx_car
            control_traj[i][4] = -path[i][2] * math.sin(control_traj[i][2]) + path[i][3] * math.cos(control_traj[i][2])                                     # vy_car
            control_traj[i+1][5] = (control_traj[i+1][2] - control_traj[i][2])/0.1                          # omega
        control_traj[-1][0] = path[-1][0]         # x
        control_traj[-1][1] = path[-1][1]         # y
        control_traj[-1][2] = math.atan2(path[-1][3], path[-1][2])         # phi
        control_traj[-1][3] = path[-1][2] * math.cos(control_traj[-1][2]) + path[-1][3] * math.sin(control_traj[-1][2])  # vx_car
        control_traj[-1][4] = -path[-1][2] * math.sin(control_traj[-1][2]) + path[-1][3] * math.cos(control_traj[-1][2])  # vy_car
        return control_traj

def pid_feedback_control(current_state, desired_state):
    e = desired_state - current_state
    kp = 20
    kv = 5
    u_ax = kp * e[0] + kv * e[2]
    u_ay = kp * e[1] + kv * e[3]
    return np.array([u_ax, u_ay])

# data storage
planed_trajectory_log = []
desired_state_log = []
actual_state_log = []

# init
t = 0.0
dt = 0.1
dt_replan = 1 # never replan
last_replan_time = t
current_state = np.array([ego_car.pos_x, ego_car.pos_y, ego_car.x_dot, ego_car.y_dot, ego_car.a_x, ego_car.a_y])  # x,y; vx,vy; ax,ay
planned_trajectory = None
u = np.array([0, 0])
u_float = np.array([0, 0])

# trajectory_decision = TrajectoryDecisionForLaneChanging(sampler)
trajectory_decision = TrajectoryDecisionForOvertaking(sampler)

while t < kSimTime:
    current_state = np.array(
        [ego_car.pos_x, ego_car.pos_y, ego_car.x_dot, ego_car.y_dot, ego_car.a_x, ego_car.a_y])  # x,y; vx,vy; ax,ay
    ## planner
    # check if need to replan
    print("t: ", t)
    print("generate plan trajectory")
    if t == 0.0 or t >= last_replan_time + dt_replan:
        ego_car_temp = ego_car
        last_replan_time = t
        planned_trajectory = trajectory_decision.trajectoryPlan(env, ego_car_temp, int(t/dt))
        ego_plan_traj = planned_trajectory.traj.trajectory

    # for i in range(100):
    #     tail_state = ego_plan_traj[-1]
    #     px = tail_state[0] + tail_state[2] * 0.1
    #     py = tail_state[1]
    #     vx = tail_state[2]
    #     vy = tail_state[3]
    #     ego_plan_traj = np.append(ego_plan_traj, np.array([[px, py, vx, vy]]), axis=0)

    # control
    desired_state = ego_plan_traj[int((t - last_replan_time) / dt)]
    desired_state_log.append(desired_state)
    actual_state_log.append(current_state)
    planed_trajectory_log.append(planned_trajectory)
    # u = pid_feedback_control(current_state, desired_state)
    # print("t: ", "{:.2f}".format(t), ", delta t: ", "{:.2f}".format(t - last_replan_time))
    # print("px: ", "{:.2f}".format(current_state[0]), "py: ", "{:.2f}".format(current_state[1]), "vx: ",
    #       "{:.2f}".format(current_state[2]), "vy: ", "{:.2f}".format(current_state[3]),
    #       "ux: ", "{:.2f}".format(u[0]), "uy: ", "{:.2f}".format(u[1]))
    # print("dpx: ", "{:.2f}".format(desired_state[0]), "dpy: ", "{:.2f}".format(desired_state[1]),
    #       "dvx: ", "{:.2f}".format(desired_state[2]), "dvy: ", "{:.2f}".format(desired_state[3]))

    # MPC
    traj_for_control = prepareStateForControl(ego_plan_traj)
    # print(ego_plan_traj)
    # print(traj_for_control)

    Q = np.eye(6)
    R = np.eye(2)
    # lane change

    # overtaking
    Q[0][0] = 5
    Q[1][1] = 10
    Q[2][2] = 2
    Q[3][3] = 4
    Q[4][4] = 5
    Q[5][5] = 1

    model = DynamicsModel(20, Q, R, traj_for_control, int((t - last_replan_time)/dt))

    # print(traj_for_control[0])
    feedback_state = [ego_car.pos_x, ego_car.pos_y, ego_car.phi, ego_car.x_dot, ego_car.y_dot, ego_car.omega]
    u = model.solve(feedback_state) # ego_car.state
    # print(u)
    u_float = np.array([float(u[0]), float(u[1])])
    print('u', u_float)
    print("last step")
    print(ego_car.pos_x, ego_car.pos_y)
    print(ego_car.phi)
    print(ego_car.v_x, ego_car.v_y)
    print(ego_car.omega)
    # print('u_float',type(u_float[0]),type(u_float[1]))


    # sim
    # ego_car.step_test(u[0], u[1])
    ego_car.step(u_float)
    for car in surrounding_car_list:
         car.step_test(0,0)

    print("next step")
    print(u_float)
    print(ego_car.pos_x, ego_car.pos_y)
    print(ego_car.phi)
    print(ego_car.v_x, ego_car.v_y)
    print(ego_car.omega)


    t += dt
print("sim over")

desired_state_log = np.array(desired_state_log)
actual_state_log = np.array(actual_state_log)
fig, ax = plt.subplots()
line1, = ax.plot(actual_state_log[:, 0], actual_state_log[:, 1], '--r')
line2, = ax.plot(desired_state_log[:, 0], desired_state_log[:, 1], '--b')
plt.legend(['actual traj', 'desired traj'])
plt.show()
plotTrajectories(ego_car, None, env, actual_state_log, desired_state_log)

debug = 0.0









