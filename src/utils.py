import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import ConvexHull
import pandas as pd
import numpy as np
import matplotlib.pylab as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
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

#
# def plotTrajectories(ego_car, ego_plan_traj, env, actual_state_log, desired_state_log):
#     actual_state_log = actual_state_log
#     desired_state_log = desired_state_log    # # trans
#     # current_state = np.array(
#     #     [ego_car.pos_x, ego_car.pos_y, ego_car.x_dot, ego_car.y_dot, ego_car.a_x, ego_car.a_y])
#     # params
#     d = env.lane.line_width
#     len_line = 300.0
#     W = ego_car.car_width
#     L = ego_car.car_legnth
#
#     ## 画场景示意图
#     fig, ax = plt.subplots()
#
#     # 画灰色路面图
#     GreyZone = np.array([[- 5, - d - 0.5], [- 5, d + 0.5], [len_line, d + 0.5], [len_line, - d - 0.5]])
#     # plt.fill(GreyZone[:, 0], GreyZone[:, 1], np.array([0.5, 0.5, 0.5]))
#     # 画分界线
#     # plt.plot(np.array([- 5, len_line]), np.array([0, 0]), 'w--')
#     # plt.plot(np.array([- 5, len_line]), np.array([d, d]), 'w')
#     # plt.plot(np.array([- 5, len_line]), np.array([- d, - d]), 'w')
#
#     # 设置坐标轴显示范围
#     plt.axis('equal')
#     plt.title("scene")
#
#     # # 画换道轨迹
#     # plt.plot(ego_plan_traj[:, 0], ego_plan_traj[:, 1], 'r--', linewidth=2.5)
#
#     # # plot ego veh
#     # plt.fill(np.array(
#     #     [current_state[0] - L / 2, current_state[0] - L / 2, current_state[0] + L / 2, current_state[0] + L / 2]),
#     #     np.array([current_state[1] - W / 2, current_state[1] + W / 2, current_state[1] + W / 2, current_state[1] - W / 2]), 'b')
#
#     # plot env veh
#     line_obs = None
#     for obs in env.surrounding_vehicle_list:
#         obs_state = np.array(
#             [obs.pos_x, obs.pos_y, obs.v_x, obs.v_y, obs.a_x, obs.a_y])
#         obs_pred_traj = np.concatenate((np.array([obs.traj_x]), np.array([obs.traj_y]))).T
#         obs_pred_traj = obs_pred_traj
#         plt.fill(np.array([obs_state[0] - L / 2, obs_state[0] - L / 2, obs_state[0] + L / 2, obs_state[0] + L / 2]),
#                  np.array([obs_state[1] - W / 2, obs_state[1] + W / 2, obs_state[1] + W / 2, obs_state[1] - W / 2]),
#                  'r')
#         line_obs, = ax.plot(obs_pred_traj[:, 0], obs_pred_traj[:, 1], 'b--')
#     # plt.show()
#     line1, = ax.plot(actual_state_log[:, 0], actual_state_log[:, 1], '--r')
#     line2, = ax.plot(desired_state_log[:, 0], desired_state_log[:, 1], '--b')
#     # plt.legend(['actual traj', 'desired traj'])
#
#     def init():
#         return line1, line2, line_obs,
#
#     def update(num):
#         begin_idx = max(num - 100, 0)
#         line1.set_data(actual_state_log[begin_idx:num, 0], actual_state_log[begin_idx:num, 1])
#         line2.set_data(desired_state_log[begin_idx:num, 0], desired_state_log[begin_idx:num, 1])
#         if line_obs is not None:
#             line_obs.set_data(obs_pred_traj[begin_idx:num, 0], obs_pred_traj[begin_idx:num, 1])
#         ax.set_xlim(actual_state_log[begin_idx, 0], actual_state_log[begin_idx + num, 0])
#         return line1, line2, line_obs,
#
#     frames = desired_state_log.shape[0]
#     ani = FuncAnimation(fig
#                         , update
#                         , init_func=init
#                         , frames=frames
#                         , interval=1
#                         , blit=True
#                         )
#     plt.show()


def plotTrajectories(ego_car, all_times_candidate_trajector_set, env, actual_state_log, desired_state_log):
    actual_state_log = actual_state_log
    desired_state_log = desired_state_log

    # Parameters
    d = env.lane.line_width
    len_line = 300.0
    W = ego_car.car_width
    L = ego_car.car_legnth  # 修改这里，改为 car_legnth
    window_size = 100  # Size of the window in x-axis

    # Plot scene
    fig, ax = plt.subplots()

    # Plot grey road surface
    GreyZone = np.array([[-5, -d - 0.5], [-5, d + 0.5], [len_line, d + 0.5], [len_line, -d - 0.5]])
    ax.fill(GreyZone[:, 0], GreyZone[:, 1], color=[0.5, 0.5, 0.5], alpha=0.5)

    # Plot lane divider lines
    ax.plot(np.array([-5, len_line]), np.array([0, 0]), 'w--')
    ax.plot(np.array([-5, len_line]), np.array([d, d]), 'w')
    ax.plot(np.array([-5, len_line]), np.array([-d, -d]), 'w')

    # Set axis to be equal
    ax.set_aspect('equal')
    plt.title("Scene")

    # Initialize lines for actual and desired trajectories
    line1, = ax.plot([], [], '--r', label='Actual Trajectory')
    line2, = ax.plot([], [], '--b', label='Desired Trajectory')

    # Plot environment vehicles
    line_obs_list = []
    rect_obs_list = []  # 用于存放他车的矩形对象
    for obs in env.surrounding_vehicle_list:
        obs_state = np.array([obs.pos_x, obs.pos_y, obs.v_x, obs.v_y, obs.a_x, obs.a_y])
        obs_pred_traj = np.concatenate((np.array([obs.traj_x]), np.array([obs.traj_y]))).T
        # 注释掉初始绘制他车矩形的代码
        # ax.fill(np.array([obs_state[0] - L / 2, obs_state[0] - L / 2, obs_state[0] + L / 2, obs_state[0] + L / 2]),
        #         np.array([obs_state[1] - W / 2, obs_state[1] + W / 2, obs_state[1] + W / 2, obs_state[1] - W / 2]),
        #         'r')
        line_obs, = ax.plot([], [], 'b--')
        line_obs_list.append((line_obs, obs_pred_traj))

        rect_obs = Rectangle((0, 0), L, W, color='red', alpha=0.7)
        rect_obs_list.append(rect_obs)
        ax.add_patch(rect_obs)

    # plt.legend()

    # Add a rectangle for ego car
    ego_car_rect = Rectangle((0, 0), L, W, color='blue', alpha=0.7)
    ax.add_patch(ego_car_rect)

    # Initialize list for candidate trajectories
    candidate_lines = []
    for i in range(all_times_candidate_trajector_set.shape[1]):
        line, = ax.plot([], [], color=np.random.rand(3, ), alpha=0.5)
        candidate_lines.append(line)

    def init():
        line1.set_data([], [])
        line2.set_data([], [])
        for line_obs, _ in line_obs_list:
            line_obs.set_data([], [])
        for rect_obs in rect_obs_list:
            rect_obs.set_xy((-L / 2, -W / 2))
        for line in candidate_lines:
            line.set_data([], [])
        ego_car_rect.set_xy((-L / 2, -W / 2))
        return line1, line2, ego_car_rect, *map(lambda x: x[0], line_obs_list), *rect_obs_list, *candidate_lines

    def update(num):
        begin_idx = max(num - window_size, 0)
        end_idx = num
        line1.set_data(actual_state_log[begin_idx:end_idx, 0], actual_state_log[begin_idx:end_idx, 1])
        line2.set_data(desired_state_log[begin_idx:end_idx, 0], desired_state_log[begin_idx:end_idx, 1])
        for i, (line_obs, obs_pred_traj) in enumerate(line_obs_list):
            line_obs.set_data(obs_pred_traj[begin_idx:end_idx, 0], obs_pred_traj[begin_idx:end_idx, 1])
            rect_obs_list[i].set_xy((obs_pred_traj[num, 0] - L / 2, obs_pred_traj[num, 1] - W / 2))
        # Dynamically update x-axis range
        ax.set_xlim(actual_state_log[begin_idx, 0], actual_state_log[begin_idx, 0] + window_size)
        # Update ego car rectangle position and rotation
        ego_car_rect.set_xy((actual_state_log[num, 0] - L / 2, actual_state_log[num, 1] - W / 2))
        ego_car_rect.angle = np.degrees(actual_state_log[num, -1])  # 更新角度

        # Update candidate trajectories
        for i, line in enumerate(candidate_lines):
            line.set_data(all_times_candidate_trajector_set[num, i, :, 0],
                          all_times_candidate_trajector_set[num, i, :, 1])

        return line1, line2, ego_car_rect, *map(lambda x: x[0], line_obs_list), *rect_obs_list, *candidate_lines

    frames = desired_state_log.shape[0]
    ani = FuncAnimation(fig, update, init_func=init, frames=frames, interval=50, blit=True)  # interval 增大到 50
    plt.show()
    ani.save('overtake.gif', dpi=300)

# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
#
# # 设置图形
# fig, ax = plt.subplots()
#
# # 初始数据
# xdata, ydata = [], []
# ln, = plt.plot([], [], 'r-', animated=True)
#
#
# # 设置初始图形属性
# def init():
#     ax.set_xlim(0, 2 * np.pi)
#     ax.set_ylim(-1, 1)
#     return ln,
#
#
# # 更新函数
# def update(frame):
#     xdata.append(frame)
#     ydata.append(np.sin(frame))
#
#     # 保持最近的100个点在图中
#     if len(xdata) > 100:
#         xdata.pop(0)
#         ydata.pop(0)
#         ax.set_xlim(xdata[0], xdata[-1])
#
#     ln.set_data(xdata, ydata)
#     return ln,
#
#
# # 创建动画
# ani = FuncAnimation(fig, update, frames=np.linspace(0, 4 * np.pi, 200),
#                     init_func=init, blit=True)
#
# # 显示动画
# plt.show()


# import matplotlib.pyplot as plt
# import matplotlib.patches as patches
# import numpy as np
#
# # 定义函数来绘制带角度的长方形
# def plot_rotated_rectangle(center, width, height, angle):
#     fig, ax = plt.subplots()
#
#     # 计算长方形左下角的点
#     rect = patches.Rectangle(
#         (center[0] - width / 2, center[1] - height / 2),  # 左下角起点
#         width,
#         height,
#         angle=angle,
#         edgecolor='r',
#         facecolor='none'
#     )
#
#     # 添加长方形到图中
#     ax.add_patch(rect)
#
#     # 设置图形的范围，以便足够显示长方形
#     buffer = max(width, height)
#     ax.set_xlim(center[0] - buffer, center[0] + buffer)
#     ax.set_ylim(center[1] - buffer, center[1] + buffer)
#     ax.set_aspect('equal', adjustable='box')
#
#     plt.grid(True)
#     plt.show()
#
# # 示例参数
# center = (2, 3)   # 长方形的中心点
# width = 4         # 长方形的宽度
# height = 2        # 长方形的高度
# angle = 45        # 长方形旋转的角度（以度为单位）
#
# # 绘制长方形
# plot_rotated_rectangle(center, width, height, angle)