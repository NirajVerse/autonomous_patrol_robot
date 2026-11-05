import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TurnTest(Node):
    def __init__(self):
        super().__init__("turn_test")

        self.pub = self.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )

        self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        self.start_yaw = None
        self.current_yaw = None
        self.finished = False

        self.timer = self.create_timer(0.05, self.control)

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation

        self.current_yaw = math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        )

        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            self.target_yaw = normalize(
                self.start_yaw + math.pi / 2
            )

    def control(self):
        if self.finished or self.current_yaw is None:
            return

        error = normalize(self.target_yaw - self.current_yaw)

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if abs(error) < 0.02:
            self.finished = True
            self.get_logger().info(
                f"Turn completed. Error: {math.degrees(error):.2f} degrees"
            )
        else:
            cmd.twist.angular.z = max(
                -0.3, min(0.3, 1.5 * error)
            )

        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = TurnTest()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop = TwistStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            node.pub.publish(stop)

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
