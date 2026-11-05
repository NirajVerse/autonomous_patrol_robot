
import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RoadController(Node):

    def __init__(self):
        super().__init__("road_controller")

        self.publisher = self.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        # Road from START to intersection A
        self.road_length = 1.3

        # Controller parameters
        self.linear_speed = 0.04
        self.k_heading = 1.5
        self.k_lateral = 2.0

        self.pose = None
        self.start_pose = None
        self.front_distance = float("inf")
        self.scan_received = False
        self.finished = False

        self.timer = self.create_timer(0.1, self.control)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

        self.pose = (p.x, p.y, yaw)

        if self.start_pose is None:
            self.start_pose = self.pose
            self.get_logger().info(
                "Initial pose recorded"
            )

    def scan_callback(self, msg):
        distances = []

        for i, r in enumerate(msg.ranges):
            angle = normalize_angle(
                msg.angle_min + i * msg.angle_increment
            )

            if abs(angle) < math.radians(20):
                if (math.isfinite(r)
                        and msg.range_min <= r <= msg.range_max):
                    distances.append(r)

        self.front_distance = min(
            distances, default=float("inf")
        )
        self.scan_received = True

    def publish_velocity(self, linear=0.0, angular=0.0):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.twist.linear.x = linear
        cmd.twist.angular.z = angular

        self.publisher.publish(cmd)

    def stop(self, message):
        self.publish_velocity()
        self.finished = True
        self.get_logger().info(message)

    def control(self):

        if self.finished or self.start_pose is None:
            return

        if not self.scan_received:
            return

        x, y, yaw = self.pose
        sx, sy, road_yaw = self.start_pose

        # Unit vector along the road
        ux = math.cos(road_yaw)
        uy = math.sin(road_yaw)

        dx = x - sx
        dy = y - sy

        # Distance traveled along road
        progress = dx * ux + dy * uy

        # Signed sideways deviation
        lateral_error = ux * dy - uy * dx

        remaining = self.road_length - progress

        # Stop when near the intersection
        if remaining <= 0.03:
            self.stop(
                "Reached intersection A"
            )
            return

        # Safety stop if estimated deviation gets too large
        if abs(lateral_error) > 0.06:
            self.stop(
                f"Stopped: lateral error "
                f"{lateral_error:.3f} m"
            )
            return

        # LiDAR obstacle check
        if self.front_distance < 0.28:
            self.stop("Obstacle ahead!")
            return

        # Heading correction plus lateral correction
        desired_yaw = road_yaw - math.atan(
            self.k_lateral * lateral_error
        )

        heading_error = normalize_angle(
            desired_yaw - yaw
        )

        angular_velocity = max(
            -0.3,
            min(0.3, self.k_heading * heading_error)
        )

        # Rotate first if heading is significantly wrong
        if abs(heading_error) > 0.15:
            linear_velocity = 0.0
        else:
            linear_velocity = min(
                self.linear_speed,
                max(0.01, 0.5 * remaining)
            )

        self.publish_velocity(
            linear_velocity,
            angular_velocity
        )


def main():
    rclpy.init()
    node = RoadController()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        pass

    finally:
        if rclpy.ok():
            node.publish_velocity()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

