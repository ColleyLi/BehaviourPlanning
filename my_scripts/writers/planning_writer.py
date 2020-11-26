from modules.planning.proto.planning_pb2 import ADCTrajectory
from modules.common.proto.pnc_point_pb2 import TrajectoryPoint
from modules.localization.proto.localization_pb2 import LocalizationEstimate

from cyber_py import cyber
import time
import sys
sys.path.append("../")

POSITION = [0.0, 0.0]


def callback(data):
    global POSITION
    if POSITION[0] < 1.0:
        POSITION = [data.pose.position.x, data.pose.position.y]
    print(POSITION)


def main():
    global POSITION
    test_node = cyber.Node("test_control_writer")

    writer = test_node.create_writer("/apollo/planning", ADCTrajectory, 6)
    test_node.create_reader(
        "/apollo/localization/pose", LocalizationEstimate, callback)

    relative_time = 0.0

    while not cyber.is_shutdown():
        time.sleep(0.02)

        trajectory = ADCTrajectory()

        trajectory_point = TrajectoryPoint()
        # POSITION[0] += 0.1
        POSITION[1] += 0.1
        trajectory_point.path_point.x = POSITION[0]
        trajectory_point.path_point.y = POSITION[1]
        trajectory_point.path_point.theta = 0.43
        trajectory_point.v = 1.0
        trajectory_point.relative_time = relative_time

        trajectory.trajectory_point.extend([trajectory_point])

        trajectory.header.timestamp_sec = time.time()

        writer.write(trajectory)

        relative_time += 0.02


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
