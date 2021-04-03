#!/usr/bin/env python

import argparse
from collections import OrderedDict

import matplotlib.pyplot as plt
import numpy as np

import common.proto_utils as proto_utils
from tools.utils import get_map, get_routing, get_center_curves, calculate_s
from tools.utils import plot_map, plot_id


def main(args):
    base_map = get_map(args.map_path)
    routing_response = get_routing(args.routing)
    center_curves = get_center_curves(args.map_path)

    plot_route(routing_response, center_curves, base_map, args.use_s)


def plot_route(routing_response, segment_center_curves, base_map, use_s=True):
    fig = plt.figure(figsize=(20, 10))
    fig.patch.set_facecolor('white')

    for road in routing_response.road:
        for passage in road.passage:
            for segment in passage.segment:
                if use_s:
                    center_x, center_y = plot_center_curve(
                        segment_center_curves[segment.id],
                        segment.start_s,
                        segment.end_s,
                    )
                else:
                    center_x, center_y = plot_center_curve(
                        segment_center_curves[segment.id])
                plot_id(center_x, center_y, str(road.id) + " " + str(segment.id))

    plt.title('Routing result')
    plt.axis('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plot_map(base_map)
    plot_waypoints(routing_response)
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = OrderedDict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())
    plt.tight_layout()
    plt.show(block=True)


def plot_center_curve(central_curves,
                      start_s=None,
                      end_s=None,
                      min_pts=100,
                      color='green'):
    node_x = []
    node_y = []
    for segment in central_curves.segment:
        px, py = proto_utils.flatten(segment.line_segment.point, ['x', 'y'])
        node_x.extend(px)
        node_y.extend(py)

    plt.gca().scatter(
        node_x,
        node_y,
        color='blue',
        alpha=0.8,
        label='Curve points')

    # Interpolate to cope with lack of points
    # Done to help find more accurate s
    if len(node_x) < min_pts and (start_s is not None or end_s is not None):
        reverse = True if node_x[0] > node_x[-1] else False

        x, y = [list(t) for t in zip(*sorted(zip(node_x, node_y)))]
        node_x = np.linspace(min(x), max(x), num=min_pts)
        node_y = np.interp(node_x, x, y)

        if reverse:
            node_x, node_y = node_x[::-1], node_y[::-1]

    start_plot_index = 0
    end_plot_index = len(node_x)
    node_s = calculate_s(node_x, node_y)

    if start_s is not None:
        for i in range(len(node_s)):
            if node_s[i] >= start_s:
                start_plot_index = i
                break

    if end_s is not None:
        for i in range(len(node_s) - 1, -1, -1):
            if node_s[i] <= end_s:
                end_plot_index = i + 1
                break

    plt.gca().plot(
        node_x[start_plot_index:end_plot_index],
        node_y[start_plot_index:end_plot_index],
        color=color,
        lw=3,
        alpha=0.8,
        label='Interpolation')

    mid_index = (start_plot_index + end_plot_index) // 2
    return node_x[mid_index], node_y[mid_index]


def plot_waypoints(routing_response):
    waypoints = routing_response.routing_request.waypoint

    start, finish = waypoints[0], waypoints[-1]

    plot_id(start.pose.x, start.pose.y, "Start", color='red')
    plot_id(finish.pose.x, finish.pose.y, "Finish", color='red')

    for waypoint in waypoints:
        circle = plt.Circle(
            (waypoint.pose.x, waypoint.pose.y), 0.3, color='red')
        plt.gca().add_artist(circle)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Route response plotter')

    parser.add_argument('--use_s',
                        action='store_true',
                        help='Whether to use s values from routing response')

    parser.add_argument('--map_path',
                        default='./data/innopolis_map/',
                        type=str,
                        help='The path to a map directory')

    parser.add_argument('--routing',
                        default='./data/routing_response.txt',
                        type=str,
                        help='The path to a routing response file')

    args = parser.parse_args()

    main(args)
