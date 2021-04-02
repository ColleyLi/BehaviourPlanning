#!/usr/bin/env python

import argparse

import matplotlib.pyplot as plt

import common.proto_utils as proto_utils
import tools.mkz_polygon as mkz_polygon
from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from tools.utils import get_map, show_map


def main(args):
    localization_pb = proto_utils.get_pb_from_text_file(
        args.localization_path, LocalizationEstimate())

    planning_pb = proto_utils.get_pb_from_text_file(
        args.planning_path, ADCTrajectory())

    fig = plt.figure(figsize=(20, 10))
    fig.patch.set_facecolor('white')

    base_map = get_map(args.map_path)
    show_map(base_map, light=True)

    plot_trajectory(planning_pb, plt)
    plot_vehicle(localization_pb, plt)

    current_t = localization_pb.header.timestamp_sec
    trajectory_point = find_closest_traj_point(planning_pb, current_t)
    plot_traj_point(planning_pb, trajectory_point, plt)

    plt.title("Current planning trajectory")
    plt.axis('equal')
    plt.tight_layout()
    plt.show()


def plot_trajectory(planning_pb, ax):
    points_x = []
    points_y = []
    points_t = []
    base_time_sec = planning_pb.header.timestamp_sec
    for trajectory_point in planning_pb.trajectory_point:
        points_x.append(trajectory_point.path_point.x)
        points_y.append(trajectory_point.path_point.y)
        points_t.append(base_time_sec + trajectory_point.relative_time)
    ax.plot(points_x, points_y, "r.")


def find_closest_t(points_t, current_t):
    if len(points_t) == 0:
        return -1
    if len(points_t) == 1:
        return points_t[0]
    if len(points_t) == 2:
        if abs(points_t[0] - current_t) < abs(points_t[1] - current_t):
            return points_t[0]
        else:
            return points_t[1]
    if points_t[len(points_t) / 2] > current_t:
        return find_closest_t(points_t[0:len(points_t) / 2], current_t)
    elif points_t[len(points_t) / 2] < current_t:
        return find_closest_t(points_t[len(points_t) / 2 + 1:], current_t)
    else:
        return current_t


def find_closest_traj_point(planning_pb, current_t):
    points_x = []
    points_y = []
    points_t = []
    base_time_sec = planning_pb.header.timestamp_sec
    for trajectory_point in planning_pb.trajectory_point:
        points_x.append(trajectory_point.path_point.x)
        points_y.append(trajectory_point.path_point.y)
        points_t.append(base_time_sec + trajectory_point.relative_time)

    matched_t = find_closest_t(points_t, current_t)
    idx = points_t.index(matched_t)
    return planning_pb.trajectory_point[idx]


def plot_traj_point(planning_pb, traj_point, ax):
    matched_t = planning_pb.header.timestamp_sec \
        + traj_point.relative_time
    ax.plot([traj_point.path_point.x], [traj_point.path_point.y], "bs")
    content = "Nearest Point\n"
    content += "Time: " + str(matched_t) + "\n"
    content += "Speed: " + str(traj_point.v) + "\n"
    content += "Acceleration: " + str(traj_point.a)
    lxy = [-80, -80]
    ax.annotate(
        content,
        xy=(traj_point.path_point.x, traj_point.path_point.y),
        xytext=lxy,
        textcoords='offset points',
        ha='left',
        va='top',
        bbox=dict(boxstyle='round,pad=0.5', fc='green', alpha=0.3),
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
        alpha=0.8)


def plot_vehicle(localization_pb, ax):
    loc_x = [localization_pb.pose.position.x]
    loc_y = [localization_pb.pose.position.y]
    current_t = localization_pb.header.timestamp_sec
    ax.plot(loc_x, loc_y, "bo")
    position = []
    position.append(localization_pb.pose.position.x)
    position.append(localization_pb.pose.position.y)
    position.append(localization_pb.pose.position.z)

    mkz_polygon.plot(position, localization_pb.pose.heading, ax)
    content = "Vechicle\n"
    content += "Time: " + str(current_t) + "\n"
    content += "Speed: " + \
        str(localization_pb.pose.linear_velocity.y) + "\n"
    content += "Acceleration: " + \
        str(localization_pb.pose.linear_acceleration_vrf.y)
    lxy = [-80, 80]
    ax.annotate(
        content,
        xy=(loc_x[0], loc_y[0]),
        xytext=lxy,
        textcoords='offset points',
        ha='left',
        va='top',
        bbox=dict(boxstyle='round,pad=0.5', fc='green', alpha=0.3),
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
        alpha=0.8)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Planning response plotter')

    parser.add_argument('--map_path',
                        default='./data/innopolis_map/',
                        type=str,
                        help='The path to a map directory')

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
