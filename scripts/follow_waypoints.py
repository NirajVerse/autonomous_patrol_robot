
import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__("waypoint_follower")

        self.cmd_pub = self.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        # Coordinates relative to initial robot orientation:
        # (forward distance, left distance), meters
        self.waypoints = [
            (1.3, 0.0),    # Center intersection
            (1.3, 1.5),    # Left intersection
            (2.8, 1.5),
        ]

        self.start_pose = None
        self.pose = None
        self.index = 0
        self.finished = False
        self.front_distance = float("inf")

        self.timer = self.create_timer(0.1, self.control)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        self.pose = (p.x, p.y, yaw)

        if self.start_pose is None:
            self.start_pose = self.pose
            self.get_logger().info("Starting waypoint navigation")

    def scan_callback(self, msg):
        front = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle = normalize_angle(angle)

            if abs(angle) < math.radians(20):
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    front.append(r)

        self.front_distance = min(front, default=float("inf"))

    def publish_cmd(self, linear=0.0, angular=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular
        self.cmd_pub.publish(cmd)

    def control(self):
        if self.start_pose is None or self.finished:
            return

        x, y, yaw = self.pose
        sx, sy, initial_yaw = self.start_pose

        forward, left = self.waypoints[self.index]

        # Convert initial robot-relative waypoint to odom frame
        target_x = sx + (
            forward * math.cos(initial_yaw)
            - left * math.sin(initial_yaw)
        )
        target_y = sy + (
            forward * math.sin(initial_yaw)
            + left * math.cos(initial_yaw)
        )

        dx = target_x - x
        dy = target_y - y
        distance = math.hypot(dx, dy)

        if distance < 0.04:
            self.publish_cmd()

            self.get_logger().info(
                f"Reached waypoint {self.index + 1}"
            )

            self.index += 1

            if self.index >= len(self.waypoints):
                self.finished = True
                self.get_logger().info("Route completed!")
            return

        desired_yaw = math.atan2(dy, dx)
        error = normalize_angle(desired_yaw - yaw)

        # Rotate in place to face the target
        if abs(error) > 0.08:
            angular = max(-0.4, min(0.4, 1.5 * error))
            self.publish_cmd(angular=angular)
            return

        # Stop if an object is too close in front
        if self.front_distance < 0.28:
            self.publish_cmd()
            self.get_logger().warn(
                "Obstacle ahead! Navigation stopped."
            )
            self.finished = True
            return

        speed = min(0.07, 0.6 * distance)
        angular = max(-0.25, min(0.25, 1.5 * error))

        self.publish_cmd(linear=speed, angular=angular)


def main():
    rclpy.init()
    node = WaypointFollower()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_cmd()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

