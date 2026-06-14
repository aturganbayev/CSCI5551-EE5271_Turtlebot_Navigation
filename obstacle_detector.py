#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ObstacleDetector(Node):
    def __init__(self):
        super().__init__("obstacle_detector")

        # QoS to match TurtleBot3 /scan
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.pub_obstacle = self.create_publisher(Bool, "obstacle", 10)

        self.sub_scan = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            scan_qos,
        )

        self.declare_parameter("distance_threshold", 0.36)
        self.declare_parameter("front_angle_deg", 36.0)

        self.distance_threshold = self.get_parameter(
            "distance_threshold"
        ).get_parameter_value().double_value

        self.front_angle_deg = self.get_parameter(
            "front_angle_deg"
        ).get_parameter_value().double_value

        self.get_logger().info("ObstacleDetector node started")

    def scan_callback(self, msg: LaserScan):
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n = len(msg.ranges)

        range_min = msg.range_min
        range_max = msg.range_max

        
        center_idx = int(round((0.0 - angle_min) / angle_inc))

        half_angle_rad = math.radians(self.front_angle_deg)
        half_count = int(round(half_angle_rad / angle_inc))

        start = max(0, center_idx - half_count)
        end = min(n - 1, center_idx + half_count)

        in_front = []
        for i in range(start, end + 1):
            r = msg.ranges[i]

            # drop NaN/inf
            if not math.isfinite(r):
                continue

            
            if r <= range_min or r >= range_max:
                continue

            
            if r < range_min + 0.02:  
                continue

            in_front.append(r)

        obstacle = False
        if in_front:
            min_dist = min(in_front)
            obstacle = min_dist < self.distance_threshold


        self.pub_obstacle.publish(Bool(data=obstacle))

def main(args=None):         
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
