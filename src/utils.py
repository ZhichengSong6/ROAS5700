import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import ConvexHull
import pandas as pd
import numpy as np
import matplotlib.pylab as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from polynomial_trajectory_planner import Polynomial

def plotVehAndPath(ego_car, ego_plan_traj, env):
    # trans
    current_state = np.array([ego_car.pos_x, ego_car.pos_y, ego_car.x_dot, ego_car.y_dot, ego_car.a_x, ego_car.a_y])  # x,y; vx,vy; ax,ay
    #
    # obs_state, obs_pred_traj,

    # params
    d = env.lane.line_width
    len_line = 100.0
    W = ego_car.car_width
    L = ego_car.car_legnth

    ## 画场景示意图
    fig, ax = plt.subplots()

    # 画灰色路面图
    GreyZone = np.array([[- 5, - d - 0.5], [- 5, d + 0.5], [len_line, d + 0.5], [len_line, - d - 0.5]])
    plt.fill(GreyZone[:, 0], GreyZone[:, 1], np.array([0.5, 0.5, 0.5]))

    # plot ego veh
    plt.fill(np.array([current_state[0] - L/2, current_state[0] - L/2, current_state[0] + L/2, current_state[0] + L/2]),
             np.array([current_state[1] - W / 2, current_state[1] + W / 2, current_state[1] + W / 2, current_state[1] - W / 2]), 'b')

    # 画分界线
    plt.plot(np.array([- 5, len_line]), np.array([0, 0]), 'w--')

    plt.plot(np.array([- 5, len_line]), np.array([d, d]), 'w')

    plt.plot(np.array([- 5, len_line]), np.array([- d, - d]), 'w')

    # 设置坐标轴显示范围
    plt.axis('equal')

    plt.title("scene")

    # 画换道轨迹
    plt.plot(ego_plan_traj[:, 0], ego_plan_traj[:, 1], 'r--', linewidth = 2.5)

    # plot env veh
    for obs in env.surrounding_vehicle_list:
        obs_state = np.array(
            [obs.pos_x, obs.pos_y, obs.v_x, obs.v_y, obs.a_x, obs.a_y])
        obs_pred_traj = np.concatenate((np.array([obs.traj_x]), np.array([obs.traj_y]))).T
        plt.fill(np.array([obs_state[0] - L / 2, obs_state[0] - L / 2, obs_state[0] + L / 2, obs_state[0] + L / 2]),
                 np.array([obs_state[1] - W / 2, obs_state[1] + W / 2, obs_state[1] + W / 2, obs_state[1] - W / 2]), 'r')
        plt.plot(obs_pred_traj[:, 0], obs_pred_traj[:, 1], 'b--')
    plt.show()


def plotTrajectories(ego_car, ego_plan_traj, env, actual_state_log, desired_state_log):
    actual_state_log = actual_state_log[::10]
    desired_state_log = desired_state_log[::10]
    # # trans
    # current_state = np.array(
    #     [ego_car.pos_x, ego_car.pos_y, ego_car.v_x, ego_car.v_y, ego_car.a_x, ego_car.a_y])
    # params
    d = env.lane.line_width
    len_line = 300.0
    W = ego_car.car_width
    L = ego_car.car_legnth

    ## 画场景示意图
    fig, ax = plt.subplots()

    # 画灰色路面图
    GreyZone = np.array([[- 5, - d - 0.5], [- 5, d + 0.5], [len_line, d + 0.5], [len_line, - d - 0.5]])
    plt.fill(GreyZone[:, 0], GreyZone[:, 1], np.array([0.5, 0.5, 0.5]))
    # 画分界线
    plt.plot(np.array([- 5, len_line]), np.array([0, 0]), 'w--')
    plt.plot(np.array([- 5, len_line]), np.array([d, d]), 'w')
    plt.plot(np.array([- 5, len_line]), np.array([- d, - d]), 'w')

    # 设置坐标轴显示范围
    # plt.axis('equal')
    plt.title("scene")

    # # 画换道轨迹
    # plt.plot(ego_plan_traj[:, 0], ego_plan_traj[:, 1], 'r--', linewidth=2.5)

    # # plot ego veh
    # plt.fill(np.array(
    #     [current_state[0] - L / 2, current_state[0] - L / 2, current_state[0] + L / 2, current_state[0] + L / 2]),
    #     np.array([current_state[1] - W / 2, current_state[1] + W / 2, current_state[1] + W / 2, current_state[1] - W / 2]), 'b')

    # plot env veh
    line_obs = None
    for obs in env.surrounding_vehicle_list:
        obs_state = np.array(
            [obs.pos_x, obs.pos_y, obs.v_x, obs.v_y, obs.a_x, obs.a_y])
        obs_pred_traj = np.concatenate((np.array([obs.traj_x]), np.array([obs.traj_y]))).T
        obs_pred_traj = obs_pred_traj[::10]
        plt.fill(np.array([obs_state[0] - L / 2, obs_state[0] - L / 2, obs_state[0] + L / 2, obs_state[0] + L / 2]),
                 np.array([obs_state[1] - W / 2, obs_state[1] + W / 2, obs_state[1] + W / 2, obs_state[1] - W / 2]),
                 'r')
        line_obs, = ax.plot(obs_pred_traj[:, 0], obs_pred_traj[:, 1], 'b--')
    # plt.show()
    line1, = ax.plot(actual_state_log[:, 0], actual_state_log[:, 1], '--r')
    line2, = ax.plot(desired_state_log[:, 0], desired_state_log[:, 1], '--b')
    # plt.legend(['actual traj', 'desired traj'])

    def init():
        return line1, line2, line_obs,

    def update(num):
        line1.set_data(actual_state_log[:num, 0], actual_state_log[:num, 1])
        line2.set_data(desired_state_log[:num, 0], desired_state_log[:num, 1])
        if line_obs is not None:
            line_obs.set_data(obs_pred_traj[:num, 0], obs_pred_traj[:num, 1])
        return line1, line2, line_obs,

    frames = desired_state_log.shape[0]
    ani = FuncAnimation(fig
                        , update
                        , init_func=init
                        , frames=frames
                        , interval=1
                        , blit=True
                        )
    # plt.show()
    ani.save("animation.gif", fps=25, writer="lane_change")

