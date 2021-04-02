#!/usr/bin/env python

import argparse

import matplotlib.pyplot as plt

from tools.utils import get_map, show_map


def main(args):
    print("Reading map data")
    base_map = get_map(args.map_path)
    print("Done reading map data")
    plot_map(base_map)


def plot_map(map):
    fig = plt.figure(figsize=(20, 10))
    fig.patch.set_facecolor('white')
    show_map(map)
    plt.axis('equal')
    plt.title("Map with road ids")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Map plotter')

    parser.add_argument('--map_path',
                        default='./data/innopolis_map/',
                        type=str,
                        help='The path to a map directory')

    args = parser.parse_args()

    main(args)
