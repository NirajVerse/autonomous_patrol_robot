
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped


def normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


class Recovery(Node):
    def __init__(self):
        super().__init__("recovery_test")

        self.pub = self.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )
        self.create_subscription(
            Odometry, "/odom", self.odom_cb, 10
        )
        self.create_subscription(
            LaserScan, "/scan", self.scan_cb, 10
        )

        # Intersection A in this simulation's odom frame
        self.target = (1.30, 0.0)

        self.pose = None
        self.front = float("inf")
        self.scan_ready = False
        self.phase = "TURN"
        self.done = False
        self.timer = self.create_timer(0.1, self.control)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )
        self.pose = (p.x, p.y, yaw)

    def scan_cb(self, msg):
        values = []
        for i, r in enumerate(msg.ranges):
            angle = normalize(
                msg.angle_min + i*msg.angle_increment
            )
            if (abs(angle) < math.radians(20)
                    and math.isfinite(r)
                    and msg.range_min <= r <= msg.range_max):
                values.append(r)

        self.front = min(values, default=float("inf"))
        self.scan_ready = True

    def command(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.pub.publish(msg)

    def finish(self, message):
        self.command()
        self.get_logger().info(message)
        self.done = True

    def control(self):
        if self.done or self.pose is None or not self.scan_ready:
            return

        x, y, yaw = self.pose
        tx, ty = self.target
        dx, dy = tx-x, ty-y
        distance = math.hypot(dx, dy)

        if distance < 0.025:
            self.finish("Returned to intersection A!")
            return

        desired = math.atan2(dy, dx)
        error = normalize(desired - yaw)

        if self.phase == "TURN":
            if abs(error) < 0.025:
                self.command()
                self.phase = "DRIVE"
                self.get_logger().info(
                    "Turn completed. Returning to A."
                )
                return

            self.command(
                angular=max(-0.20, min(0.20, 1.2*error))
            )
            return

        # Stop if something blocks the return direction
        if self.front < 0.25:
            self.finish("Obstacle ahead during recovery!")
            return

        # Re-align if heading error grows
        angular = max(-0.20, min(0.20, 1.5*error))
        linear = min(0.035, 0.4*distance)

        if abs(error) > 0.15:
            linear = 0.0

        self.command(linear, angular)


def main():
    rclpy.init()
    node = Recovery()

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):
                node.command()
                rclpy.spin_once(node, timeout_sec=0.05)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
