from cyber_py import cyber
from modules.routing.proto.routing_pb2 import RoutingResponse

import sys
sys.path.append("../")


def callback(data):
    print("-----------")
    print(data)


def main():
    test_node = cyber.Node("routing_reader")
    test_node.create_reader("/apollo/routing_response",
                            RoutingResponse, callback)
    test_node.spin()


if __name__ == '__main__':

    cyber.init()
    main()
    cyber.shutdown()
