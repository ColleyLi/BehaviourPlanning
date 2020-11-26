#!/usr/bin/env python

import os
import math

import gflags
import matplotlib.pyplot as plt

import common.proto_utils as proto_utils
import modules.map.proto.map_pb2 as map_pb2
import modules.routing.proto.topo_graph_pb2 as topo_graph_pb2
import modules.routing.proto.routing_pb2 as routing_pb2

FLAGS = gflags.FLAGS
gflags.DEFINE_string(
    'map_dir', 'modules/map/data/innopolis_map', 'map directory')

COLORS = [
    'navy',
    'c',
    'cornflowerblue',
    'gold',
    'darkorange',
    'darkviolet',
    'aquamarine',
    'firebrick',
    'limegreen'
]


def show_map(drivemap, light=False):
    road_lane_set = []
    for road in drivemap.road:
        lanes = []
        for sec in road.section:
            lanes.extend(proto_utils.flatten(sec.lane_id, 'id'))
        road_lane_set.append(lanes)

    for lane in drivemap.lane:
        for curve in lane.central_curve.segment:
            if curve.HasField('line_segment'):
                road_idx = get_road_index_of_lane(lane.id.id, road_lane_set)
                if not light:
                    plot_line(curve.line_segment,
                              COLORS[road_idx % len(COLORS)])

                if road_idx == -1:
                    print('Failed to get road index of lane')
                    continue

                center_x, center_y = get_lane_center(curve.line_segment)
                if not light:
                    plot_id(center_x, center_y, "road_" + str(road_idx))

        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                plot_boundary(curve.line_segment)

        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                plot_boundary(curve.line_segment)


def get_road_index_of_lane(lane_id, road_lane_set):
    for i, lane_set in enumerate(road_lane_set):
        if lane_id in lane_set:
            return i
    return -1


def plot_line(line_segment, color='green'):
    px, py = proto_utils.flatten(line_segment.point, ['x', 'y'])
    px, py = downsample_array(px), downsample_array(py)
    plt.gca().plot(px, py, lw=10, alpha=0.8, color=color)
    return px[len(px) // 2], py[len(py) // 2]


def calculate_s(px, py):
    s = 0.0
    ps = [s]
    for i in range(len(px) - 1):
        s += math.sqrt(pow(px[i + 1] - px[i], 2) + pow(py[i + 1] - py[i], 2))
        ps.append(s)
    return ps


def get_lane_center(line_segment):
    px, py = proto_utils.flatten(line_segment.point, ['x', 'y'])
    return px[len(px) // 2], py[len(py) // 2]


def get_center_curves(map_dir):
    graph = get_topology(map_dir)

    center_curves = {}
    for node in graph.node:
        center_curves[node.lane_id] = node.central_curve

    return center_curves


def get_topology(map_dir):
    print('Please wait for loading routing topo data...')
    topo_data_path = os.path.join(map_dir, 'routing_map.bin')
    print('Topo File: %s' % topo_data_path)
    return proto_utils.get_pb_from_bin_file(topo_data_path,
                                            topo_graph_pb2.Graph())


def get_map(map_dir):
    print('Please wait for loading map data...')
    map_data_path = os.path.join(map_dir, 'base_map.bin')
    print('Map File: %s' % map_data_path)
    return proto_utils.get_pb_from_bin_file(map_data_path, map_pb2.Map())


def get_routing(routing_file):
    print('Please wait for loading route response data...')
    print("Routing File: %s" % routing_file)
    return proto_utils.get_pb_from_text_file(routing_file,
                                             routing_pb2.RoutingResponse())


def onclick(event):
    print('\nClick captured! x=%f\ty=%f' % (event.xdata, event.ydata))
    print('cmd>')


def plot_map(mapfile):
    for lane in mapfile.lane:
        for curve in lane.left_boundary.curve.segment:
            if curve.HasField('line_segment'):
                plot_boundary(curve.line_segment)

        for curve in lane.right_boundary.curve.segment:
            if curve.HasField('line_segment'):
                plot_boundary(curve.line_segment)


def plot_boundary(line_segment):
    px, py = proto_utils.flatten(line_segment.point, ['x', 'y'])
    px, py = downsample_array(px), downsample_array(py)
    plt.gca().plot(px, py, 'k', lw=0.4)


def downsample_array(array, step=5):
    result = array[::step]
    result.append(array[-1])
    return result


def plot_id(x, y, id_string, color='green'):
    plt.annotate(
        id_string,
        xy=(x, y),
        xytext=(40, -40),
        textcoords='offset points',
        ha='right',
        va='bottom',
        bbox=dict(boxstyle='round,pad=0.3', fc=color, alpha=0.5),
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
