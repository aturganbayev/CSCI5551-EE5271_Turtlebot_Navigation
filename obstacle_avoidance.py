#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance")

        self.obstacle = False

        self.sub_obstacle = self.create_subscription(
            Bool, "obstacle", self.obstacle_callback, 10
        )
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)

        self.declare_parameter("turn_speed", -0.6)
        self.turn_speed = (
            self.get_parameter("turn_speed").get_parameter_value().double_value
        )

        # timer at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

        self.get_logger().info("ObstacleAvoidance node started")

    def obstacle_callback(self, msg: Bool):
        self.obstacle = msg.data

    def control_loop(self):
        
        if self.obstacle:
            twist = Twist()
            twist.linear.x = -0.03   
            twist.angular.z = self.turn_speed  
            self.pub_cmd.publish(twist)
        


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
