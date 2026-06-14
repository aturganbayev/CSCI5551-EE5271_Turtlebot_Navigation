#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from tf2_ros import Buffer, TransformListener, TransformException


class ArucoFollowerLidar(Node):


    def __init__(self):
        super().__init__("aruco_follower_lidar")

        # ---------- Parameters ----------
        self.declare_parameter("marker_id", 0)
        self.declare_parameter("target_distance", 0.5)     # [m] desired distance
        self.declare_parameter("min_distance", 0.2)        # [m] absolute safety minimum
        self.declare_parameter("linear_kp", 0.7)
        self.declare_parameter("angular_kp", 2.0)
        self.declare_parameter("max_linear_speed", 0.2)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("lookup_timeout_sec", 0.05)
        self.declare_parameter("max_tf_age_sec", 0.3)      # marker TF max age
        self.declare_parameter("front_angle_deg", 15.0)    # LiDAR cone +/- angle

        self.marker_id = self.get_parameter("marker_id").get_parameter_value().integer_value
        self.target_distance = self.get_parameter("target_distance").get_parameter_value().double_value
        self.min_distance = self.get_parameter("min_distance").get_parameter_value().double_value
        self.linear_kp = self.get_parameter("linear_kp").get_parameter_value().double_value
        self.angular_kp = self.get_parameter("angular_kp").get_parameter_value().double_value
        self.max_linear_speed = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self.max_angular_speed = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self.lookup_timeout = Duration(
            seconds=self.get_parameter("lookup_timeout_sec").get_parameter_value().double_value
        )
        self.max_tf_age_sec = self.get_parameter("max_tf_age_sec").get_parameter_value().double_value
        self.front_angle_deg = self.get_parameter("front_angle_deg").get_parameter_value().double_value

        # ---------- TF listener ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- LiDAR subscriber ----------
        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            scan_qos,
        )
        self.last_scan: Optional[LaserScan] = None

        # ---------- cmd_vel publisher ----------
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Control loop timer (20 Hz)
        self.timer = self.create_timer(1.0 / 20.0, self.control_loop)

        self.get_logger().info(
            f"ArucoFollowerLidar started. Following ar_marker_{self.marker_id}, "
            f"target distance = {self.target_distance:.2f} m"
        )

    # ========== Callbacks ==========

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    # ========== Helper functions ==========

    def get_heading_error(self) -> Optional[float]:
        """
        Return yaw error (rad) to marker in base_link frame.
        None if no valid / recent TF.
        """
        marker_frame = f"ar_marker_{self.marker_id}"
        base_frame = "base_link"

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                base_frame,
                marker_frame,
                now,
                timeout=self.lookup_timeout,
            )
        except TransformException as ex:
            self.get_logger().debug(f"TF lookup failed for {marker_frame}: {ex}")
            return None

        # Reject stale transforms
        tf_stamp = Time.from_msg(trans.header.stamp)
        now_ros = self.get_clock().now()
        age = (now_ros - tf_stamp).nanoseconds / 1e9

        if age > self.max_tf_age_sec:
            self.get_logger().debug(
                f"Marker TF too old (age={age:.2f}s > {self.max_tf_age_sec:.2f}s)"
            )
            return None

        # Translation in base_link frame
        tx = trans.transform.translation.x  # forward
        ty = trans.transform.translation.y  # left (+) / right (-)

        # yaw error: angle between robot forward (x) and marker direction
        yaw_error = math.atan2(ty, tx)
        return yaw_error

    def get_front_distance(self) -> Optional[float]:
        """
        Return min distance in a front cone from LiDAR.
        None if no scan or no valid ranges.
        """
        if self.last_scan is None:
            return None

        msg = self.last_scan
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        n = len(msg.ranges)

        half_angle_rad = math.radians(self.front_angle_deg)
        center_idx = int(round((0.0 - angle_min) / angle_inc))
        half_count = int(round(half_angle_rad / angle_inc))

        start = max(0, center_idx - half_count)
        end = min(n - 1, center_idx + half_count)

        dists = [
            r for r in msg.ranges[start : end + 1]
            if math.isfinite(r)
        ]
        if not dists:
            return None

        return min(dists)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())

    # ========== Main control loop ==========

    def control_loop(self):
        yaw_error = self.get_heading_error()
        front_dist = self.get_front_distance()

        # If we lost marker or have no LiDAR data → stop
        if yaw_error is None or front_dist is None:
            self.publish_stop()
            return

        # ---- Angular control: align to marker ----
        angular_cmd = -self.angular_kp * yaw_error  # minus: turn toward marker

        # ---- Linear control: LiDAR distance ----
        distance_error = front_dist - self.target_distance

        # Safety: too close -> back up a bit
        if front_dist < self.min_distance:
            linear_cmd = -0.05  # gentle backup
            self.get_logger().debug("Too close! Backing up slightly.")
        else:
            linear_cmd = self.linear_kp * distance_error

        # Clamp speeds
        linear_cmd = max(-self.max_linear_speed, min(self.max_linear_speed, linear_cmd))
        angular_cmd = max(-self.max_angular_speed, min(self.max_angular_speed, angular_cmd))

        # Deadzones to avoid jitter
        if abs(distance_error) < 0.02:
            linear_cmd = 0.0
        if abs(yaw_error) < math.radians(1.0):
            angular_cmd = 0.0

        # Debug (enable by setting logger level to DEBUG if you like)
        self.get_logger().debug(
            f"front_dist={front_dist:.3f}, dist_err={distance_error:.3f}, "
            f"yaw_err={math.degrees(yaw_error):.1f}deg, "
            f"lin={linear_cmd:.3f}, ang={angular_cmd:.3f}"
        )

        twist = Twist()
        twist.linear.x = float(linear_cmd)
        twist.angular.z = float(angular_cmd)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoFollowerLidar()
    try:
        rclpy.spin(node)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
