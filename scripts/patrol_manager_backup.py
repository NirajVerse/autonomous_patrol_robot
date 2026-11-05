
import math
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped

from route_planner import nodes, find_route


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class PatrolManager(Node):

    def __init__(self):
        super().__init__("patrol_manager")

        self.pub = self.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        self.stations = ["P1", "P2", "P3"]
        self.station_index = 0
        self.current_node = "START"

        self.route = []
        self.segment_index = 0
        self.phase = "ROTATE"

        self.pose = None
        self.initial_odom = None
        self.scan_received = False
        self.front_distance = float("inf")

        self.world_start = (0.0, -1.3095, math.pi / 2)

        # Feedback controller parameters
        self.k_heading = 1.5
        self.k_lateral = 2.0

        self.max_linear = 0.04
        self.max_angular = 0.20

        self.arrival_tolerance = 0.01
        self.max_lateral_error = 0.06

        self.finished = False
        self.report = {}

        self.timer = self.create_timer(0.1, self.control)

        self.get_logger().info("Patrol manager ready")

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
            self.get_logger().info(
                "Initial odometry recorded"
            )

    def scan_callback(self, msg):
        front = []

        for i, r in enumerate(msg.ranges):
            angle = normalize(
                msg.angle_min + i * msg.angle_increment
            )

            if abs(angle) < math.radians(20):
                if (math.isfinite(r)
                        and msg.range_min <= r <= msg.range_max):
                    front.append(r)

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
        ox, oy, initial_yaw = self.initial_odom
        sx, sy, syaw = self.world_start

        dx = wx - sx
        dy = wy - sy

        rotation = normalize(initial_yaw - syaw)

        return (
            ox + dx*math.cos(rotation)
               - dy*math.sin(rotation),
            oy + dx*math.sin(rotation)
               + dy*math.cos(rotation)
        )

    def stop(self, reason):
        self.publish_cmd()
        self.finished = True

        self.get_logger().warn(reason)
        self.print_report()

    def print_report(self):
        self.get_logger().info("===== PATROL REPORT =====")

        for station in self.stations:
            status = self.report.get(
                station, "NOT VISITED"
            )
            self.get_logger().info(
                f"{station}: {status}"
            )

        self.get_logger().info("=========================")

    def plan_next_station(self):
        if self.station_index >= len(self.stations):
            self.publish_cmd()
            self.finished = True
            self.get_logger().info("Patrol completed!")
            self.print_report()
            return

        destination = self.stations[self.station_index]

        path = find_route(
            self.current_node, destination
        )

        if path is None:
            self.stop(
                f"No route to {destination}"
            )
            return

        # Keep complete path as consecutive edges
        self.route = path
        self.segment_index = 0
        self.phase = "ROTATE"

        self.get_logger().info(
            f"Route to {destination}: "
            + " -> ".join(path)
        )

    def complete_segment(self, end_name, lateral):
        self.publish_cmd()

        self.current_node = end_name

        self.get_logger().info(
            f"Reached {end_name}, "
            f"lateral error={lateral:.3f} m"
        )

        self.segment_index += 1
        self.phase = "ROTATE"

        # More road segments remain
        if self.segment_index < len(self.route) - 1:
            return

        # Patrol station reached
        station = self.stations[self.station_index]

        self.report[station] = "VISITED"

        self.get_logger().info(
            f"{station}: VISITED"
        )

        self.station_index += 1
        self.route = []

    def control(self):
        if self.finished or self.pose is None:
            return

        if self.initial_odom is None or not self.scan_received:
            return

        if not self.route:
            self.plan_next_station()
            return

        start_name = self.route[self.segment_index]
        end_name = self.route[self.segment_index + 1]

        sx, sy = self.world_to_odom(
            *nodes[start_name]
        )
        ex, ey = self.world_to_odom(
            *nodes[end_name]
        )

        x, y, yaw = self.pose

        # Road direction
        road_dx = ex - sx
        road_dy = ey - sy

        road_length = math.hypot(
            road_dx, road_dy
        )

        ux = road_dx / road_length
        uy = road_dy / road_length

        road_yaw = math.atan2(
            road_dy, road_dx
        )

        # Robot position relative to segment start
        dx = x - sx
        dy = y - sy

        # Along-road progress
        progress = dx*ux + dy*uy

        # Signed distance from road centerline
        lateral_error = ux*dy - uy*dx

        remaining = road_length - progress

        # Obstacle safety check
        if self.front_distance < 0.28:
            self.stop(
                f"Obstacle detected on "
                f"{start_name} -> {end_name}"
            )
            return

        # Safety: excessive cross-track error
        if abs(lateral_error) > self.max_lateral_error:
            self.stop(
                f"Road deviation too large: "
                f"{lateral_error:.3f} m"
            )
            return

        # PHASE 1: Rotate toward road direction
        if self.phase == "ROTATE":
            heading_error = normalize(
                road_yaw - yaw
            )

            if abs(heading_error) < 0.015:
                self.publish_cmd()
                self.phase = "DRIVE"

                self.get_logger().info(
                    f"Aligned: {start_name} -> {end_name}"
                )
                return

            angular = max(
                -self.max_angular,
                min(
                    self.max_angular,
                    self.k_heading * heading_error
                )
            )

            self.publish_cmd(angular=angular)
            return

        # PHASE 2: End of road segment
        if remaining <= self.arrival_tolerance:
            self.complete_segment(
                end_name, lateral_error
            )
            return

        # PHASE 3: Centerline feedback
        desired_yaw = road_yaw - math.atan(
            self.k_lateral * lateral_error
        )

        heading_error = normalize(
            desired_yaw - yaw
        )

        angular = max(
            -self.max_angular,
            min(
                self.max_angular,
                self.k_heading * heading_error
            )
        )

        if abs(heading_error) > 0.15:
            linear = 0.0
        else:
            linear = min(
                self.max_linear,
                max(0.01, 0.5 * remaining)
            )

        self.publish_cmd(linear, angular)


def main():
    rclpy.init()
    node = PatrolManager()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("Patrol interrupted")

    finally:
        if rclpy.ok():
            node.publish_cmd()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

