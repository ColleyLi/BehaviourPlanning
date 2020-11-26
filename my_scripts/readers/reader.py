from cyber_py import cyber
from modules.control.proto.control_cmd_pb2 import ControlCommand

import sys
sys.path.append("../")


def callback(data):
    print("-----------")
    print(data)


def main():
    test_node = cyber.Node("listener")
    test_node.create_reader("/apollo/control", ControlCommand, callback)
    test_node.spin()


if __name__ == '__main__':

    cyber.init()
    main()
    cyber.shutdown()
