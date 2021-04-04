import argparse
import os
import sys
import time

from cyber_py import cyber
from cyber_py import cyber_time
from modules.routing.proto import routing_pb2

def main():
    """
    Main rosnode
    """
    node = cyber.Node("routing_request_writer")
    sequence_num = 0

    routing_request = routing_pb2.RoutingRequest()

    routing_request.header.module_name = 'routing_request'

    waypoint = routing_request.waypoint.add()
    waypoint.pose.x = 358519.69714355469
    waypoint.pose.y = 6180750.5960693359
    waypoint.id = 'lane_410'
    waypoint.s = 22

    waypoint = routing_request.waypoint.add()
    waypoint.pose.x = 358548.46653527877
    waypoint.pose.y = 6180741.2346008308
    waypoint.id = 'lane_414'
    waypoint.s = 10

    writer = node.create_writer('/apollo/routing_request',
                                routing_pb2.RoutingRequest)

    time.sleep(2)

    routing_request.header.timestamp_sec = time.time()
    routing_request.header.sequence_num = sequence_num
    writer.write(routing_request)
    
    while not cyber.is_shutdown():
        time.sleep(1)



if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()