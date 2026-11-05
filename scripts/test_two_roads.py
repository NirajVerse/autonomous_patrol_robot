
import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TwoRoadTest(Node):
    def __init__(self):
        super().__init__("two_road_test")

        self.pub = self.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        # Gazebo world coordinates
        self.nodes = {
            "START": (0.0, -1.3),
            "A": (0.0, 0.0),
            "B": (-1.5, 0.0),
        }

        self.route = [("START", "A"), ("A", "B")]
        self.segment_index = 0
        self.phase = "ROTATE"

        self.initial_odom = None
        self.pose = None
        self.front_distance = float("inf")
        self.scan_received = False
        self.finished = False

        self.k_heading = 1.5
        self.k_lateral = 2.0
        self.max_linear = 0.04
        self.max_angular = 0.30

        self.timer = self.create_timer(0.1, self.control)

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2 * (q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

        self.pose = (p.x, p.y, yaw)

        if self.initial_odom is None:
            self.initial_odom = self.pose
            self.get_logger().info("Initial pose recorded")

    def scan_callback(self, msg):
        front = []

        for i, distance in enumerate(msg.ranges):
            angle = normalize(
                msg.angle_min + i * msg.angle_increment
            )

            if abs(angle) < math.radians(20):
                if (math.isfinite(distance)
                        and msg.range_min <= distance <= msg.range_max):
                    front.append(distance)

        self.front_distance = min(
            front, default=float("inf")
        )
        self.scan_received = True

    def publish_cmd(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def world_to_odom(self, wx, wy):
        # Expected Gazebo start: (0, -1.3), facing north
        ox, oy, initial_yaw = self.initial_odom

        dx = wx
        dy = wy + 1.3
        rotation = initial_yaw - math.pi / 2

        return (
            ox + dx*math.cos(rotation) - dy*math.sin(rotation),
            oy + dx*math.sin(rotation) + dy*math.cos(rotation)
        )

    def stop(self, message):
        self.publish_cmd()
        self.finished = True
        self.get_logger().info(message)

    def control(self):
        if self.finished or self.pose is None:
            return

        if self.initial_odom is None or not self.scan_received:
            return

        if self.segment_index >= len(self.route):
            self.stop("Two-road test completed!")
            return

        start_name, end_name = self.route[self.segment_index]
        start = self.world_to_odom(*self.nodes[start_name])
        end = self.world_to_odom(*self.nodes[end_name])

        x, y, yaw = self.pose
        sx, sy = start
        ex, ey = end

        road_dx = ex - sx
        road_dy = ey - sy
        road_length = math.hypot(road_dx, road_dy)

        road_yaw = math.atan2(road_dy, road_dx)
        ux = road_dx / road_length
        uy = road_dy / road_length

        dx = x - sx
        dy = y - sy

        progress = dx*ux + dy*uy
        lateral_error = ux*dy - uy*dx
        remaining = road_length - progress

        if self.front_distance < 0.28:
            self.stop("Obstacle ahead. Test stopped.")
            return

        # STEP 1: Rotate toward the road direction
        if self.phase == "ROTATE":
            error = normalize(road_yaw - yaw)

            if abs(error) < 0.035:
                self.publish_cmd()
                self.phase = "DRIVE"
                self.get_logger().info(
                    f"Aligned: {start_name} -> {end_name}"
                )
                return

            angular = max(
                -self.max_angular,
                min(self.max_angular, self.k_heading*error)
            )

            self.publish_cmd(angular=angular)
            return

        # STEP 2: Check whether segment is finished
        if remaining <= 0.03:
            self.publish_cmd()

            self.get_logger().info(
                f"Reached {end_name}, "
                f"lateral error={lateral_error:.3f} m"
            )

            self.segment_index += 1
            self.phase = "ROTATE"

            if self.segment_index >= len(self.route):
                self.stop("Reached B. Test completed!")
            return

        # Safety threshold for this simulation test
        if abs(lateral_error) > 0.07:
            self.stop(
                f"Excessive lateral deviation: "
                f"{lateral_error:.3f} m"
            )
            return

        # STEP 3: Cross-track feedback
        desired_yaw = road_yaw - math.atan(
            self.k_lateral * lateral_error
        )

        heading_error = normalize(desired_yaw - yaw)

        angular = max(
            -self.max_angular,
            min(self.max_angular,
                self.k_heading * heading_error)
        )

        if abs(heading_error) > 0.15:
            linear = 0.0
        else:
            linear = min(
                self.max_linear,
                max(0.01, 0.5*remaining)
            )

        self.publish_cmd(linear, angular)


def main():
    rclpy.init()
    node = TwoRoadTest()

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
