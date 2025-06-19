#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import random
import math

# Constants
WORKSPACE_WIDTH = 100
WORKSPACE_HEIGHT = 100
NUM_UAVS = 10
COVERAGE_RADIUS = 10  # Each UAV covers a circular area of this radius


class UAV:
    def __init__(self, x, y, id):
        self.x = x
        self.y = y
        self.id = id


class UAVPlacerNode(Node):
    def __init__(self):
        super().__init__("uav_placer_node")
        self.get_logger().info("UAV Placer Node started.")
        self.place_uavs()

    def place_uavs(self):
        uavs = self.generate_random_uavs(NUM_UAVS, COVERAGE_RADIUS)
        self.get_logger().info("Placed UAVs at:")
        for uav in uavs:
            self.get_logger().info(f"UAV-{uav.id}: ({uav.x:.2f}, {uav.y:.2f})")

        intersecting_pairs = self.check_intersections(uavs, COVERAGE_RADIUS)
        self.get_logger().info("Intersecting UAV pairs (overlapping coverage):")
        for pair in intersecting_pairs:
            self.get_logger().info(f"UAV-{pair[0]} <--> UAV-{pair[1]}")

    def generate_random_uavs(self, num_uavs, radius):
        uavs = []
        for i in range(num_uavs):
            x = random.uniform(radius, WORKSPACE_WIDTH - radius)
            y = random.uniform(radius, WORKSPACE_HEIGHT - radius)
            uavs.append(UAV(x, y, i))
        return uavs

    def check_intersections(self, uavs, radius):
        pairs = []
        for i in range(len(uavs)):
            for j in range(i + 1, len(uavs)):
                dist = math.hypot(uavs[i].x - uavs[j].x, uavs[i].y - uavs[j].y)
                if dist < 2 * radius:
                    pairs.append((uavs[i].id, uavs[j].id))
        return pairs


def main(args=None):
    rclpy.init(args=args)
    node = UAVPlacerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()