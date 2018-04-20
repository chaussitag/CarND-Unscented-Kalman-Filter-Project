#!/usr/bin/env python
# coding=utf-8

import argparse
import os
import sys

import matplotlib.pyplot as plt


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="visualize the NIS for lidar and radar")
    parser.add_argument("--log_file", "-i", required=True,
                        help="path to the log file contains NIS values for lidar and radar")

    args = parser.parse_args()
    if not os.path.isfile(args.log_file):
        print("the log file %s does not exist" % args.log_file)
        sys.exit(-1)

    radar_nis = []
    lidar_nis = []
    with open(args.log_file, "r") as f:
        for line in f:
            if line.startswith("NIS:"):
                line_splits = line.strip().split()
                sensor_type = line_splits[1]
                if sensor_type == 'radar':
                    radar_nis.append(float(line_splits[2]))
                elif sensor_type == 'lidar':
                    lidar_nis.append(float(line_splits[2]))

    fig, (axes1, axes2) = plt.subplots(1, 2, sharey=True, figsize=(25, 10))

    axes1.set_ylim(top=15)
    axes1.grid(True)
    axes1.set_title("Radar NIS")
    radar_x = range(0, len(radar_nis))
    axes1.plot(radar_x, radar_nis)
    axes1.plot(radar_x, [7.815] * len(radar_nis))
    axes1.text(0, 7.815, r"7.815", color='red', fontsize=15)

    axes2.grid(True)
    axes2.set_title("Lidar NIS")
    lidar_x = range(0, len(lidar_nis))
    axes2.plot(lidar_x, lidar_nis)
    axes2.plot(lidar_x, [7.815] * len(lidar_nis))
    axes2.text(0, 7.815, r"7.815", color='red', fontsize=15)

    plt.show()
