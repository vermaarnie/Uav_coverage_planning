#!/usr/bin/env python3
import json
import random
import rclpy
from rclpy.node import Node
from my_uav_interfaces.srv import GenerateUAV


class UAVService(Node):
    def __init__(self):
        super().__init__('uav_service')
        self.srv = self.create_service(GenerateUAV, 'generate_uavs', self.handle_request)
        self.get_logger().info(" UAV service ready on 'generate_uavs'")

    def handle_request(self, request, response):
        count = request.count
        self.get_logger().info(f"Received request to generate {count} UAVs.")

        uavs = []
        screen_width = 500
        screen_height = 500
        radius = 50

        for i in range(count):
            x = random.randint(radius, screen_width - radius)
            y = random.randint(radius, screen_height - radius)
            uavs.append({
                "id": i,
                "x": x,
                "y": y,
                "radius": radius
            })

        response.json_uav_list = json.dumps(uavs)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = UAVService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
