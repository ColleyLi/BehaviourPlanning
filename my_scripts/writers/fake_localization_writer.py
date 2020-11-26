from modules.perception.proto.perception_obstacle_pb2 import (
    PerceptionObstacles,
)
from modules.localization.proto.imu_pb2 import CorrectedImu
from modules.drivers.gnss.proto.ins_pb2 import InsStat
from modules.localization.proto.gps_pb2 import Gps
from modules.canbus.proto.chassis_pb2 import Chassis

from cyber_py import cyber
import time
import sys
sys.path.append("../")

COMPLETE_AUTO_DRIVE = 1
GEAR_DRIVE = 1


def main():
    test_node = cyber.Node("fake_sim_writer")

    obstacle_writer = test_node.create_writer(
        "/apollo/perception/obstacles", PerceptionObstacles, 6)

    odometry_writer = test_node.create_writer(
        "/apollo/sensor/gnss/odometry", Gps, 6)

    status_writer = test_node.create_writer(
        "/apollo/sensor/gnss/ins_stat", InsStat, 6)

    imu_writer = test_node.create_writer(
        "/apollo/sensor/gnss/corrected_imu", CorrectedImu, 6)

    chassis_writer = test_node.create_writer(
        "/apollo/canbus/chassis", Chassis, 6)

    obstacles_msg = PerceptionObstacles()

    odometry_msg = Gps()

    # Right lane (2 lanes)
    odometry_msg.localization.position.x = 358504.72
    odometry_msg.localization.position.y = 6180757.72
    odometry_msg.localization.position.z = 0.84

    # Left lane (2 lanes)
    # odometry_msg.localization.position.x = 358508.72
    # odometry_msg.localization.position.y = 6180759.0
    # odometry_msg.localization.position.z = 0.84

    # Right lane (3 lanes)
    # odometry_msg.localization.position.x = 358706.12
    # odometry_msg.localization.position.y = 6180665.23
    # odometry_msg.localization.position.z = 3.02

    odometry_msg.localization.orientation.qx = 0.005095433
    odometry_msg.localization.orientation.qy = -0.011954642
    odometry_msg.localization.orientation.qz = -0.843348324
    odometry_msg.localization.orientation.qw = 0.537210107

    odometry_msg.localization.linear_velocity.x = 0.0
    odometry_msg.localization.linear_velocity.y = 0.0
    odometry_msg.localization.linear_velocity.z = 0.0

    odometry_msg.localization.heading = -115

    status_msg = InsStat()

    status_msg.ins_status = 3
    status_msg.pos_type = 56

    imu_msg = CorrectedImu()

    imu_msg.imu.linear_acceleration.x = 0.21
    imu_msg.imu.linear_acceleration.y = -0.14
    imu_msg.imu.linear_acceleration.z = 0.807

    imu_msg.imu.angular_velocity.x = 0.0
    imu_msg.imu.angular_velocity.y = 0.0
    imu_msg.imu.angular_velocity.z = 0.0

    imu_msg.imu.heading = -115

    imu_msg.imu.euler_angles.x = -6.2617
    imu_msg.imu.euler_angles.y = -6.2684
    imu_msg.imu.euler_angles.z = -2.0071

    chassis_msg = Chassis()

    chassis_msg.engine_started = 1
    chassis_msg.engine_rpm = 800.0
    chassis_msg.speed_mps = 0.0
    chassis_msg.brake_percentage = 100.0
    chassis_msg.driving_mode = COMPLETE_AUTO_DRIVE
    chassis_msg.gear_location = GEAR_DRIVE

    chassis_msg.header.module_name = 'chassis'

    chassis_msg.chassis_gps.year = 1970
    chassis_msg.chassis_gps.month = 1
    chassis_msg.chassis_gps.day = 1
    chassis_msg.chassis_gps.hours = 3
    chassis_msg.chassis_gps.compass_direction = 135.0
    chassis_msg.chassis_gps.pdop = 0.0
    chassis_msg.chassis_gps.hdop = 0.0
    chassis_msg.chassis_gps.vdop = 0.0
    chassis_msg.chassis_gps.heading = 115
    chassis_msg.chassis_gps.num_satellites = 15
    chassis_msg.chassis_gps.gps_speed = 0.0

    sequence_num = 0
    while not cyber.is_shutdown():
        time.sleep(0.1)

        timestamp = time.time()

        obstacles_msg.header.timestamp_sec = timestamp
        odometry_msg.header.timestamp_sec = timestamp - 0.1
        status_msg.header.timestamp_sec = timestamp - 0.1
        imu_msg.header.timestamp_sec = timestamp
        chassis_msg.header.timestamp_sec = timestamp

        odometry_msg.header.sequence_num = sequence_num

        obstacle_writer.write(obstacles_msg)
        odometry_writer.write(odometry_msg)
        status_writer.write(status_msg)
        imu_writer.write(imu_msg)
        chassis_writer.write(chassis_msg)

        sequence_num += 1


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()
