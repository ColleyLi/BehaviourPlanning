from modules.perception.proto.perception_obstacle_pb2 import PerceptionObstacles, PerceptionObstacle

from cyber_py import cyber
import time
import math
import sys
sys.path.append("../")

# apollo.perception.PerceptionObstacles

VEHICLE = 5

def main():
    obstacles_msg = PerceptionObstacles()

    obstacle = PerceptionObstacle()
    obstacle.id = 10
    obstacle.type = VEHICLE
    obstacle.length = 4
    obstacle.width = 2
    obstacle.height = 1
    obstacle.position.x = 358530.72
    obstacle.position.y = 6180745.5
    obstacle.position.z = 0.84
    obstacle.theta = -0.4
    speed = 0.0
    obstacle.velocity.x = speed * math.cos(obstacle.theta)
    obstacle.velocity.y = speed * math.sin(obstacle.theta)
    obstacle.velocity.z = 0.0
    obstacles_msg.perception_obstacle.extend([obstacle])

    # Obstacle (2 lanes left near)
    obstacle = PerceptionObstacle()
    obstacle.id = 123
    obstacle.type = VEHICLE
    obstacle.length = 4
    obstacle.width = 2
    obstacle.height = 1
    obstacle.position.x = 358508.72
    obstacle.position.y = 6180759.0
    obstacle.position.z = 0.84
    obstacle.theta = -0.4
    speed = 3.0
    obstacle.velocity.x = speed * math.cos(obstacle.theta)
    obstacle.velocity.y = speed * math.sin(obstacle.theta)
    obstacle.velocity.z = 0.0
    obstacles_msg.perception_obstacle.extend([obstacle])

    # obstacle = PerceptionObstacle()
    # obstacle.id = 11
    # obstacle.type = VEHICLE
    # obstacle.length = 4
    # obstacle.width = 2
    # obstacle.height = 1
    # obstacle.position.x = 358580.72
    # obstacle.position.y = 6180726.5
    # obstacle.position.z = 0.84
    # obstacle.theta = -0.4
    # speed = 0.0
    # obstacle.velocity.x = speed * math.cos(obstacle.theta)
    # obstacle.velocity.y = speed * math.sin(obstacle.theta)
    # obstacle.velocity.z = 0.0
    # obstacles_msg.perception_obstacle.extend([obstacle]) 

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
