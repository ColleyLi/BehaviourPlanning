from cyber_py import cyber
from modules.planning.proto.planning_pb2 import ADCTrajectory

import sys
sys.path.append("../")


def callback(data):
    print("-----------")
    print(data)


def main():
    test_node = cyber.Node("routing_reader")
    test_node.create_reader("/apollo/planning",
                            ADCTrajectory, callback)
    test_node.spin()


if __name__ == '__main__':

    cyber.init()
    main()
    cyber.shutdown()
