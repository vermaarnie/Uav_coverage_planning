#!/usr/bin/env python3
from itertools import combinations
import rclpy
from rclpy.node import Node
from my_uav_interfaces.srv import GenerateUAV
import json
import math


class UAVClient(Node):
    def __init__(self):
        super().__init__('uav_client')
        self.cli = self.create_client(GenerateUAV, 'generate_uavs')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(' Waiting for service...')
        self.req = GenerateUAV.Request()

    def send_request(self, count):
        self.req.count = count
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            uav_list = json.loads(future.result().json_uav_list)
            print("\n Received UAVs:")
            for uav in uav_list:
                print(f"  ID: {uav['id']} | X: {uav['x']} | Y: {uav['y']} | R: {uav['radius']}")
            self.check_intersections(uav_list)
        else:
            self.get_logger().error(" Service call failed")
    def check_intersections(self, uavs):
        print("\n🔍 Checking for intersections:")
        found = False

        # Check all unique pairs using combinations
        for uav1, uav2 in combinations(uavs, 2):
            x1, y1, r1 = uav1["x"], uav1["y"], uav1["radius"]
            x2, y2, r2 = uav2["x"], uav2["y"], uav2["radius"]

            distance = math.hypot(x2 - x1, y2 - y1)

            if distance < (r1 + r2):
                print(f" UAV {uav1['id']} intersects with UAV {uav2['id']}")
                found = True

        if not found:
            print(" No intersections found.")


def main(args=None):
    rclpy.init(args=args)
    node = UAVClient()
    node.send_request(count=10)  # CHANGE THIS VALUE AS NEEDED
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
