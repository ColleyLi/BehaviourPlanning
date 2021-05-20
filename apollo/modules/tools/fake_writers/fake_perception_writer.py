import time
import math

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time

from modules.perception.proto.perception_obstacle_pb2 import (
    PerceptionObstacles,
    PerceptionObstacle
)

VEHICLE = 5

def main():
    test_node = cyber.Node("fake_perception_writer")

    obstacle_writer = test_node.create_writer(
        "/apollo/perception/obstacles", PerceptionObstacles, 6)

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
    # speed = 0.0
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
    # speed = 0.0
    # obstacle.velocity.x = speed * math.cos(obstacle.theta)
    # obstacle.velocity.y = speed * math.sin(obstacle.theta)
    # obstacle.velocity.z = 0.0
    # obstacles_msg.perception_obstacle.extend([obstacle])

    # Obstacle (2 lanes left front far)
    obstacle = PerceptionObstacle()
    obstacle.id = 121
    obstacle.type = VEHICLE
    obstacle.length = 4
    obstacle.width = 2
    obstacle.height = 1
    obstacle.position.x = 358545.72
    obstacle.position.y = 6180742.5
    obstacle.position.z = 0.84
    obstacle.velocity.x = 0.0
    obstacle.velocity.y = 0.0
    obstacle.velocity.z = 0.0
    obstacle.theta = -0.4
    obstacles_msg.perception_obstacle.extend([obstacle])

    # Obstacle (2 lanes right front far)
    obstacle = PerceptionObstacle()
    obstacle.id = 120
    obstacle.type = VEHICLE
    obstacle.length = 4
    obstacle.width = 2
    obstacle.height = 1
    obstacle.position.x = 358544.72
    obstacle.position.y = 6180739.5
    obstacle.position.z = 0.84
    speed = 0.0
    obstacle.velocity.x = speed * math.cos(obstacle.theta)
    obstacle.velocity.y = speed * math.sin(obstacle.theta)
    obstacle.velocity.z = 0.0
    obstacle.theta = -0.4
    obstacles_msg.perception_obstacle.extend([obstacle])

    # Obstacle (2 lanes right front medium)
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
    # speed = 0.0
    # obstacle.velocity.x = speed * math.cos(obstacle.theta)
    # obstacle.velocity.y = speed * math.sin(obstacle.theta)
    # obstacle.velocity.z = 0.0
    # obstacles_msg.perception_obstacle.extend([obstacle])

    sequence_num = 0
    while not cyber.is_shutdown():
        time.sleep(0.1)

        timestamp = cyber_time.Time.now().to_sec()

        obstacles_msg.header.timestamp_sec = timestamp

        obstacle_writer.write(obstacles_msg)

        sequence_num += 1


if __name__ == '__main__':
    cyber.init()
    main()
    cyber.shutdown()