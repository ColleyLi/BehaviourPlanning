from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles

from cyber_py import cyber
import time
import sys
sys.path.append("../")

# apollo.perception.PerceptionObstacles


def main():
    obstacles_msg = PerceptionObstacles()
    test_node = cyber.Node("test_perception_writer")

    writer = test_node.create_writer(
        "/apollo/perception/obstacles", PerceptionObstacles, 6)

    while not cyber.is_shutdown():
        time.sleep(0.01)

        obstacles_msg.header.timestamp_sec = time.time()

        writer.write(obstacles_msg)


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
