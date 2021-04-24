import os
import sys
import time

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.perception.proto.perception_obstacle_pb2 import (
    PerceptionObstacles,
    # PerceptionObstacle
)
from modules.localization.proto.imu_pb2 import CorrectedImu
from modules.drivers.gnss.proto.ins_pb2 import InsStat
from modules.localization.proto.gps_pb2 import Gps
from modules.localization.proto.localization_pb2 import LocalizationStatus 
from modules.localization.proto.localization_pb2 import LocalizationEstimate
from modules.canbus.proto.chassis_pb2 import Chassis

COMPLETE_AUTO_DRIVE = 1
GEAR_DRIVE = 1
VEHICLE = 5

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

    localization_estimate_writer = test_node.create_writer(
        "/apollo/localization/pose", LocalizationEstimate, 6)

    localization_status_writer = test_node.create_writer(
        "/apollo/localization/msf_status", LocalizationStatus, 6)

    obstacles_msg = PerceptionObstacles()

    # Obstacle (2 lanes left near)
    # obstacle = PerceptionObstacle()
    # obstacle.id = 123
    # obstacle.type = VEHICLE
    # obstacle.length = 4
    # obstacle.width = 2
    # obstacle.height = 1
    # obstacle.position.x = 358508.72
    # obstacle.position.y = 6180759.0
    # obstacle.position.z = 0.84
    # obstacle.theta = -0.4
    # speed = 3.0
    # obstacle.velocity.x = speed * math.cos(obstacle.theta)
    # obstacle.velocity.y = speed * math.sin(obstacle.theta)
    # obstacle.velocity.z = 0.0
    # obstacles_msg.perception_obstacle.extend([obstacle])

    # Obstacle (2 lanes right front close)
    # obstacle = PerceptionObstacle()
    # obstacle.id = 122
    # obstacle.type = VEHICLE
    # obstacle.length = 4
    # obstacle.width = 2
    # obstacle.height = 1
    # obstacle.position.x = 358515.72
    # obstacle.position.y = 6180752.5
    # obstacle.position.z = 0.84
    # obstacle.theta = -0.4
    # speed = 1.0
    # obstacle.velocity.x = speed * math.cos(obstacle.theta)
    # obstacle.velocity.y = speed * math.sin(obstacle.theta)
    # obstacle.velocity.z = 0.0
    # obstacles_msg.perception_obstacle.extend([obstacle])

    # Obstacle (2 lanes left front far)
    # obstacle = PerceptionObstacle()
    # obstacle.id = 121
    # obstacle.type = VEHICLE
    # obstacle.length = 4
    # obstacle.width = 2
    # obstacle.height = 1
    # obstacle.position.x = 358545.72
    # obstacle.position.y = 6180742.5
    # obstacle.position.z = 0.84
    # obstacle.velocity.x = 0.0
    # obstacle.velocity.y = 0.0
    # obstacle.velocity.z = 0.0
    # obstacle.theta = -0.4
    # obstacles_msg.perception_obstacle.extend([obstacle])

    # Obstacle (2 lanes right front far)
    # obstacle = PerceptionObstacle()
    # obstacle.id = 120
    # obstacle.type = VEHICLE
    # obstacle.length = 4
    # obstacle.width = 2
    # obstacle.height = 1
    # obstacle.position.x = 358545.72
    # obstacle.position.y = 6180742.5
    # obstacle.position.z = 0.84
    # speed = 3.0
    # obstacle.velocity.x = speed * math.cos(obstacle.theta)
    # obstacle.velocity.y = speed * math.sin(obstacle.theta)
    # obstacle.velocity.z = 0.0
    # obstacle.theta = -0.4
    # obstacles_msg.perception_obstacle.extend([obstacle])

    # # Obstacle (2 lanes right front medium)
    # obstacle = PerceptionObstacle()
    # obstacle.id = 119
    # obstacle.type = VEHICLE
    # obstacle.length = 4
    # obstacle.width = 2
    # obstacle.height = 1
    # obstacle.position.x = 358530.72
    # obstacle.position.y = 6180745.5
    # obstacle.position.z = 0.84
    # obstacle.theta = -0.4
    # speed = 3.0
    # obstacle.velocity.x = speed * math.cos(obstacle.theta)
    # obstacle.velocity.y = speed * math.sin(obstacle.theta)
    # obstacle.velocity.z = 0.0
    # obstacles_msg.perception_obstacle.extend([obstacle])

    # odometry_msg = Gps()

    # After crossroad before the bus stop
    # odometry_msg.localization.position.x = 358694.2
    # odometry_msg.localization.position.y = 6180674.0
    # odometry_msg.localization.position.z = 204.4

    # Right side
    # odometry_msg.localization.position.x = 358627.72
    # odometry_msg.localization.position.y = 6180717.72
    # odometry_msg.localization.position.z = 0.84 

    # Right lane (2 lanes)
    # odometry_msg.localization.position.x = 358504.72
    # odometry_msg.localization.position.y = 6180757.72
    # odometry_msg.localization.position.z = 0.84

    # Left lane (2 lanes)
    # odometry_msg.localization.position.x = 358508.72
    # odometry_msg.localization.position.y = 6180759.0
    # odometry_msg.localization.position.z = 0.84

    # Between lanes (2 lanes)
    # odometry_msg.localization.position.x = 358504.72
    # odometry_msg.localization.position.y = 6180759.0
    # odometry_msg.localization.position.z = 0.84

    # Right lane (3 lanes)
    # odometry_msg.localization.position.x = 358706.12
    # odometry_msg.localization.position.y = 6180665.23
    # odometry_msg.localization.position.z = 3.02

    # odometry_msg.localization.orientation.qx = 0.005095433
    # odometry_msg.localization.orientation.qy = -0.011954642
    # odometry_msg.localization.orientation.qz = -0.843348324
    # odometry_msg.localization.orientation.qw = 0.537210107

    # odometry_msg.localization.linear_velocity.x = 0.0
    # odometry_msg.localization.linear_velocity.y = 0.0
    # odometry_msg.localization.linear_velocity.z = 0.0

    # odometry_msg.localization.heading = -115

    # status_msg = InsStat()

    # status_msg.ins_status = 3
    # status_msg.pos_type = 56

    # imu_msg = CorrectedImu()

    # imu_msg.imu.linear_acceleration.x = 0.21
    # imu_msg.imu.linear_acceleration.y = -0.14
    # imu_msg.imu.linear_acceleration.z = 0.807

    # imu_msg.imu.angular_velocity.x = 0.0
    # imu_msg.imu.angular_velocity.y = 0.0
    # imu_msg.imu.angular_velocity.z = 0.0

    # imu_msg.imu.heading = -115

    # imu_msg.imu.euler_angles.x = -6.2617
    # imu_msg.imu.euler_angles.y = -6.2684
    # imu_msg.imu.euler_angles.z = -2.0071

    # chassis_msg = Chassis()

    # chassis_msg.engine_started = 1
    # chassis_msg.engine_rpm = 800.0
    # chassis_msg.speed_mps = 0.0
    # chassis_msg.brake_percentage = 100.0
    # chassis_msg.driving_mode = COMPLETE_AUTO_DRIVE
    # chassis_msg.gear_location = GEAR_DRIVE

    # chassis_msg.header.module_name = 'chassis'

    # chassis_msg.chassis_gps.year = 1970
    # chassis_msg.chassis_gps.month = 1
    # chassis_msg.chassis_gps.day = 1
    # chassis_msg.chassis_gps.hours = 3
    # chassis_msg.chassis_gps.compass_direction = 135.0
    # chassis_msg.chassis_gps.pdop = 0.0
    # chassis_msg.chassis_gps.hdop = 0.0
    # chassis_msg.chassis_gps.vdop = 0.0
    # chassis_msg.chassis_gps.heading = 115
    # chassis_msg.chassis_gps.num_satellites = 15
    # chassis_msg.chassis_gps.gps_speed = 0.0


    localization_msg = LocalizationEstimate()
    localization_msg.pose.position.x = 358621.64
    localization_msg.pose.position.y = 6180720.4
    localization_msg.pose.position.z = 203

    localization_msg.pose.orientation.qx = 0.000681112
    localization_msg.pose.orientation.qy = -0.005984223
    localization_msg.pose.orientation.qz = 0.542498648
    localization_msg.pose.orientation.qw = 0.840035141
    
    localization_msg.pose.linear_velocity.x = 0.0
    localization_msg.pose.linear_velocity.y = 0.0
    localization_msg.pose.linear_velocity.z = 0.0
    
    localization_msg.pose.linear_acceleration.x = 0.0
    localization_msg.pose.linear_acceleration.y = 0.0
    localization_msg.pose.linear_acceleration.z = 9.81
    
    localization_msg.pose.angular_velocity.x = 0.0
    localization_msg.pose.angular_velocity.y = 0.0
    localization_msg.pose.angular_velocity.z = 0.0

    localization_msg.pose.heading = 2.7176

    localization_msg.pose.linear_acceleration_vrf.x = 0.0
    localization_msg.pose.linear_acceleration_vrf.y = 0.0
    
    localization_status = LocalizationStatus()
    localization_status.fusion_status = 0
    localization_status.state_message = "" 

    sequence_num = 0
    while not cyber.is_shutdown():
        time.sleep(0.1)

        timestamp = time.time()

        obstacles_msg.header.timestamp_sec = timestamp
        # odometry_msg.header.timestamp_sec = timestamp
        # status_msg.header.timestamp_sec = timestamp
        # imu_msg.header.timestamp_sec = timestamp
        # chassis_msg.header.timestamp_sec = timestamp
        localization_msg.header.timestamp_sec = timestamp
        localization_status.header.timestamp_sec = timestamp
        localization_status.measurement_time = timestamp

        # odometry_msg.header.sequence_num = sequence_num
        localization_msg.header.sequence_num = sequence_num

        obstacle_writer.write(obstacles_msg)
        localization_estimate_writer.write(localization_msg)
        localization_status_writer.write(localization_status)
        # odometry_writer.write(odometry_msg)
        # status_writer.write(status_msg)
        # imu_writer.write(imu_msg)
        # chassis_writer.write(chassis_msg)

        sequence_num += 1


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()