
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from scipy.spatial import ConvexHull
import pandas as pd
import numpy as np
import matplotlib.pylab as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
from polynomial_trajectory_planner import Polynomial
from utils import plotVehAndPath

kMaxCost = 1e8

class EvaluatedTrajectory:
    def __init__(self, traj):
        self.id = None
        self.traj = traj
        self.lc_time = None
        self.safety_cost = None
        self.efficiency_cost = None
        self.comfort_cost = None
        self.inlane_cost = None
        self.total_cost = None

class TrajectoryDecisionForLaneChanging:
    def __init__(self, sampler):
        self.sampler = sampler

    def trajectorySample(self, ego_car, lane):
        trajectory_samples = []

        ego_car_speed = ego_car.v_x
        lc_time = 4.0
        current_state = np.array([ego_car.pos_x, ego_car.pos_y, ego_car.x_dot, ego_car.y_dot, ego_car.a_x, ego_car.a_y])  # x,y; vx,vy; ax,ay
        # sample for 20 trajectories for each time
        sample_resolution = 1.0
        kMinDeltaSpeed = -3.0
        kMaxDeltaSpeed =  8.0
        delta_speeds = np.arange(kMinDeltaSpeed, kMaxDeltaSpeed, sample_resolution)
        end_speeds = ego_car_speed + delta_speeds
        for end_speed in end_speeds:
            end_pos_x = self.sampler.computeEndPosition(ego_car.pos_x, ego_car.v_x, end_speed, lc_time)
            end_state = np.array([end_pos_x, lane.line_width / 2, end_speed, 0, 0, 0])
            poly_for_ego_car = Polynomial(lc_time)
            ego_plan_traj = poly_for_ego_car.getTrajectory(current_state, end_state)
            # poly_for_ego_car.plotPath(ego_plan_traj, current_state, end_state, lane.line_width, 35, ego_car.car_width,
            #                           ego_car.car_legnth)
            trajectory_samples.append(poly_for_ego_car)
        # dummy_poly = Polynomial(1.0)
        # dummy_poly.plotAllSamplePath(trajectory_samples, current_state, lane.line_width, 35, ego_car.car_width,
        #                              ego_car.car_legnth)
        return trajectory_samples

    def evaluateSafety(self, env, trajectory, t_start):
        collision = False
        safey_cost = 0.0
        k_min_dist = 0.2

        time_stamp = t_start
        for state in trajectory.trajectory:
            phi = 0.0 #np.arctan()
            ego_front_center_x = state[0] + env.ego_vehicle.r * np.cos(phi)
            ego_front_center_y = state[1] + env.ego_vehicle.r * np.sin(phi)
            ego_rear_center_x = state[0] - env.ego_vehicle.r * np.cos(phi)
            ego_rear_center_y = state[1] - env.ego_vehicle.r * np.sin(phi)
            ego_front_center = np.array([ego_front_center_x, ego_front_center_y])
            ego_rear_center = np.array([ego_rear_center_x, ego_rear_center_y])
            min_obs_dist = 10000.0
            for obs in env.surrounding_vehicle_list:
                obs_x = obs.traj_x[time_stamp]
                obs_y = obs.traj_y[time_stamp]
                # todo:
                obs_front_center = np.array([obs_x + obs.r, obs_y])
                obs_rear_center = np.array([obs_x - obs.r, obs_y])
                collision, obs_dist = env.pointCollisionCheck(ego_front_center, obs_front_center, ego_rear_center, obs_rear_center)
                if collision:
                    break
                min_obs_dist = min(min_obs_dist, obs_dist)
            safey_cost += min(1.0, np.exp(-(min_obs_dist - k_min_dist)))

            if collision:
                safey_cost = kMaxCost
                break
            time_stamp += 1
            safey_cost = safey_cost / len(trajectory.trajectory)
        # print("safey_cost: ", safey_cost)
        return safey_cost


    def evaluateEfficiency(self, env, trajectory):
        efficiency_cost = 0.0
        desired_speed = env.ego_vehicle.desired_speed
        trajectory_end_state = trajectory.trajectory[-1]
        trajectory_end_speed = trajectory_end_state[2]
        delta_speed_on_desired = trajectory_end_speed - desired_speed

        if delta_speed_on_desired > 0.0:
            # speed up lane change
            efficiency_cost = - np.exp(-0.02 * abs(delta_speed_on_desired)) + 1
        else:
            # slow down lane change
            efficiency_cost = - np.exp(-0.25 * abs(delta_speed_on_desired)) + 1
        return efficiency_cost

    def computeTotalCost(self, evaluated_trajectory):
        kSafetyCoeff = 0.8
        kEfficiencyCoeff = 0.2
        total_cost = (kSafetyCoeff * evaluated_trajectory.safety_cost +
                                           kEfficiencyCoeff * evaluated_trajectory.efficiency_cost)
        return total_cost

    def trajectoryEvaluator(self, env, trajectory_samples, t_start):
        evaluated_trajector_set = []
        idx = 0
        for trajectory in trajectory_samples:
            evaluated_trajectory = EvaluatedTrajectory(trajectory)
            # plotVehAndPath(ego_car, trajectory.trajectory, env)
            # simulate()
            evaluated_trajectory.id = idx
            evaluated_trajectory.safety_cost = self.evaluateSafety(env, trajectory, t_start)
            evaluated_trajectory.efficiency_cost = self.evaluateEfficiency(env, trajectory)
            # evaluated_trajectory.comfort_cost = evaluateComfort()
            evaluated_trajectory.total_cost = self.computeTotalCost(evaluated_trajectory)
            evaluated_trajector_set.append(evaluated_trajectory)
            idx += 1
        return evaluated_trajector_set

    def trajectorySelector(self, env, evaluated_trajector_set):
        selected_trajectory = None
        min_cost = kMaxCost
        for evaluated_trajectory in evaluated_trajector_set:
            if evaluated_trajectory.total_cost < min_cost:
                selected_trajectory = evaluated_trajectory
                min_cost = evaluated_trajectory.total_cost
        print("end_state: ", selected_trajectory.traj.trajectory[-1, 0], ", idx: ", selected_trajectory.id)
        # plotVehAndPath(env.ego_vehicle, selected_trajectory.traj.trajectory, env)
        return selected_trajectory


    def trajectoryPlan(self, env, ego_car, t_start):
        trajectory_samples = self.trajectorySample(ego_car, env.lane)
        evaluated_trajector_set = self.trajectoryEvaluator(env, trajectory_samples, t_start)
        selected_trajectory = self.trajectorySelector(env, evaluated_trajector_set)
        return selected_trajectory
    

class TrajectoryDecisionForOvertaking:
    def __init__(self, sampler):
        self.sampler = sampler

    def trajectorySample(self, ego_car, lane):
        trajectory_samples = []

        ego_car_speed = ego_car.v_x
        lc_time = 4.0
        current_state = np.array([ego_car.pos_x, ego_car.pos_y, ego_car.x_dot, ego_car.y_dot, ego_car.a_x, ego_car.a_y])  # x,y; vx,vy; ax,ay
        # sample for 20 trajectories for each time
        sample_resolution = 1
        kMinDeltaSpeed = 0
        kMaxDeltaSpeed =  7
        delta_speeds = np.arange(kMinDeltaSpeed, kMaxDeltaSpeed, sample_resolution)
        end_speeds = ego_car_speed + delta_speeds
        end_ys = np.arange(-3, 2, 0.5)

        for end_speed in end_speeds:
            for end_y in end_ys:
                end_pos_x = self.sampler.computeEndPosition(ego_car.pos_x, ego_car.v_x, end_speed, lc_time)
                end_state = np.array([end_pos_x, end_y, end_speed, 0, 0, 0])
                poly_for_ego_car = Polynomial(lc_time)
                ego_plan_traj = poly_for_ego_car.getTrajectory(current_state, end_state)
                # poly_for_ego_car.plotPath(ego_plan_traj, current_state, end_state, lane.line_width, 35, ego_car.car_width,
                #                           ego_car.car_legnth)
                trajectory_samples.append(poly_for_ego_car)
            # dummy_poly = Polynomial(1.0)
        # dummy_poly.plotAllSamplePath(trajectory_samples, current_state, lane.line_width, 35, ego_car.car_width,
        #                              ego_car.car_legnth)
        return trajectory_samples

    def evaluateSafety(self, env, trajectory, t_start):
        collision = False
        safey_cost = 0.0
        k_min_dist = 0.2

        time_stamp = t_start
        for state in trajectory.trajectory:
            phi = 0.0 #np.arctan()
            ego_front_center_x = state[0] + env.ego_vehicle.r * np.cos(phi)
            ego_front_center_y = state[1] + env.ego_vehicle.r * np.sin(phi)
            ego_rear_center_x = state[0] - env.ego_vehicle.r * np.cos(phi)
            ego_rear_center_y = state[1] - env.ego_vehicle.r * np.sin(phi)
            ego_front_center = np.array([ego_front_center_x, ego_front_center_y])
            ego_rear_center = np.array([ego_rear_center_x, ego_rear_center_y])
            min_obs_dist = 10000.0
            for obs in env.surrounding_vehicle_list:
                obs_x = obs.traj_x[time_stamp]
                obs_y = obs.traj_y[time_stamp]
                # todo:
                obs_front_center = np.array([obs_x + obs.r, obs_y])
                obs_rear_center = np.array([obs_x - obs.r, obs_y])
                collision, obs_dist = env.pointCollisionCheck(ego_front_center, obs_front_center, ego_rear_center, obs_rear_center)
                if collision:
                    break
                min_obs_dist = min(min_obs_dist, obs_dist)
            safey_cost += min(1.0, np.exp(-(min_obs_dist - k_min_dist)))

            if collision:
                safey_cost = kMaxCost
                break
            time_stamp += 1
            safey_cost = safey_cost / len(trajectory.trajectory)
        # print("safey_cost: ", safey_cost)
        return safey_cost


    def evaluateEfficiency(self, env, trajectory):
        efficiency_cost = 0.0
        desired_speed = env.ego_vehicle.desired_speed
        trajectory_end_state = trajectory.trajectory[-1]
        trajectory_end_speed = trajectory_end_state[2]
        delta_speed_on_desired = trajectory_end_speed - desired_speed

        if delta_speed_on_desired > 0.0:
            # speed up lane change
            efficiency_cost = - np.exp(-0.02 * abs(delta_speed_on_desired)) + 1
        else:
            # slow down lane change
            efficiency_cost = - np.exp(-0.25 * abs(delta_speed_on_desired)) + 1
        return efficiency_cost

    def evaluateInLaneCost(self, env, trajectory):
        inlane_cost = 0
        for i in range(len(trajectory.trajectory)):
            inlane_cost += abs(-2 - trajectory.trajectory[i][1])/len(trajectory.trajectory)
        return inlane_cost


    def computeTotalCost(self, evaluated_trajectory):
        kSafetyCoeff = 0.7
        kEfficiencyCoeff = 0.1
        kInLaneCoeff = 0.2
        total_cost = (kSafetyCoeff * evaluated_trajectory.safety_cost +
                                           kEfficiencyCoeff * evaluated_trajectory.efficiency_cost + kInLaneCoeff * evaluated_trajectory.inlane_cost)
        return total_cost

    def trajectoryEvaluator(self, env, trajectory_samples, t_start):
        evaluated_trajector_set = []
        idx = 0
        for trajectory in trajectory_samples:
            evaluated_trajectory = EvaluatedTrajectory(trajectory)
            # plotVehAndPath(ego_car, trajectory.trajectory, env)
            # simulate()
            evaluated_trajectory.id = idx
            evaluated_trajectory.safety_cost = self.evaluateSafety(env, trajectory, t_start)
            evaluated_trajectory.efficiency_cost = self.evaluateEfficiency(env, trajectory)
            # evaluated_trajectory.comfort_cost = evaluateComfort()
            evaluated_trajectory.inlane_cost = self.evaluateInLaneCost(env, trajectory)
            evaluated_trajectory.total_cost = self.computeTotalCost(evaluated_trajectory)
            evaluated_trajector_set.append(evaluated_trajectory)
            idx += 1
        return evaluated_trajector_set

    def trajectorySelector(self, env, evaluated_trajector_set):
        selected_trajectory = None
        min_cost = kMaxCost
        for evaluated_trajectory in evaluated_trajector_set:
            if evaluated_trajectory.total_cost < min_cost:
                min_cost = evaluated_trajectory.total_cost
                selected_trajectory = evaluated_trajectory
        print("end_state: ", selected_trajectory.traj.trajectory[-1, 1], ", idx: ", selected_trajectory.id)
        # plotVehAndPath(env.ego_vehicle, selected_trajectory.traj.trajectory, env)
        return selected_trajectory


    def trajectoryPlan(self, env, ego_car, t_start):
        trajectory_samples = self.trajectorySample(ego_car, env.lane)
        evaluated_trajector_set = self.trajectoryEvaluator(env, trajectory_samples, t_start)
        selected_trajectory = self.trajectorySelector(env, evaluated_trajector_set)
        candidate_trajector_set = []
        for evaluated_trajector in evaluated_trajector_set:
            candidate_trajector_set.append(evaluated_trajector.traj.trajectory)
        candidate_trajector_set = np.array(candidate_trajector_set)
        return selected_trajectory, candidate_trajector_set