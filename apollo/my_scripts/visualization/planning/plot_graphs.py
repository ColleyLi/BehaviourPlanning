#!/usr/bin/env python

import argparse

import matplotlib.pyplot as plt

import common.proto_utils as proto_utils
from modules.planning.proto.planning_pb2 import ADCTrajectory


def main(args):
    planning_pb = proto_utils.get_pb_from_text_file(
        args.planning_path, ADCTrajectory())

    result = get_data(planning_pb)
    plot_result(result, relative_time=args.relative_time)


def get_data(planning_pb):
    points_x = []
    points_y = []
    points_theta = []
    points_kappa = []
    points_s = []
    points_t = []
    points_t_rel = []
    points_v = []
    points_a = []

    base_time_sec = planning_pb.header.timestamp_sec
    for trajectory_point in planning_pb.trajectory_point:
        points_x.append(trajectory_point.path_point.x)
        points_y.append(trajectory_point.path_point.y)
        points_theta.append(trajectory_point.path_point.theta)
        points_kappa.append(trajectory_point.path_point.kappa)
        points_s.append(trajectory_point.path_point.s)
        points_t_rel.append(trajectory_point.relative_time)
        points_t.append(base_time_sec + trajectory_point.relative_time)
        points_v.append(trajectory_point.v)
        points_a.append(trajectory_point.a)

    result = {
        't': points_t,
        't_relative': points_t_rel,
        'x': points_x,
        'y': points_y,
        's': points_s,
        'theta': points_theta,
        'kappa': points_kappa,
        'v': points_v,
        'a': points_a,
    }

    return result


def plot_result(result, relative_time=True):
    if relative_time:
        time = result['t_relative']
    else:
        time = result['t']

    width = 2
    color = "#b45eff"

    fig = plt.figure(figsize=(20, 10))
    fig.patch.set_facecolor('white')

    plt.subplot(2, 3, 1)
    plt.plot(time, result['s'], linewidth=width, color=color)
    plt.title('S vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel("S (m)")

    plt.subplot(2, 3, 2)
    plt.plot(time, result['kappa'], linewidth=width, color=color)
    plt.title('Curvature vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel("Kappa (m^-1)")

    plt.subplot(2, 3, 3)
    plt.plot(time, result['theta'], linewidth=width, color=color)
    plt.title('Heading vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel("Theta (rad)")

    plt.subplot(2, 3, 4)
    plt.scatter(result['x'], result['y'], linewidth=width, color=color, s=2)
    plt.title('Path')
    plt.xlabel('X (m)')
    plt.ylabel("Y (m)")

    plt.subplot(2, 3, 5)
    plt.plot(time, result['v'], linewidth=width, color=color)
    plt.title('Velocity vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel("Velocity (m/s)")

    plt.subplot(2, 3, 6)
    plt.plot(time, result['a'], linewidth=width, color=color)
    plt.title('Acceleration vs Time')
    plt.xlabel('Time (s)')
    plt.ylabel("Acceleration (m/s^2)")

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Planning response plotter')

    parser.add_argument('--relative_time',
                        action='store_true',
                        help='Whether to use relative time')

    parser.add_argument('--localization_path',
                        default='./data/localization.txt',
                        type=str,
                        help='The path to a localization file')

    parser.add_argument('--planning_path',
                        default='./data/adc_trajectory.txt',
                        type=str,
                        help='The path to a planning trajectory file')

    args = parser.parse_args()

    main(args)
