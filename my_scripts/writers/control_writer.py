from modules.control.proto.control_cmd_pb2 import ControlCommand

from cyber_py import cyber
import time
import sys
sys.path.append("../")


def main():
    control_cmd = ControlCommand()
    test_node = cyber.Node("test_control_writer")

    writer = test_node.create_writer("/apollo/control", ControlCommand, 6)

    control_cmd.throttle = 30.0
    control_cmd.brake = 0.0
    # control_cmd.steering_rate = 100.0
    control_cmd.steering_target = 0.0
    # control_cmd.gear_location = Chassis.GEAR_DRIVE

    while not cyber.is_shutdown():
        time.sleep(0.1)

        control_cmd.header.timestamp_sec = time.time()

        writer.write(control_cmd)


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
