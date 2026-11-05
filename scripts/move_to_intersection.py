import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class MoveToIntersection(Node):
    def __init__(self):
        super().__init__('move_to_intersection')

        self.publisher = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.start_x = None
        self.start_y = None
        self.current_x = None
        self.current_y = None

        self.target_distance = 1.3
        self.finished = False

        self.timer = self.create_timer(0.1, self.control)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info("Starting movement")

    def control(self):
        if self.start_x is None or self.finished:
            return

        distance = math.hypot(
            self.current_x - self.start_x,
            self.current_y - self.start_y
        )

        remaining = self.target_distance - distance

        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()

        if remaining > 0.03:
            cmd.twist.linear.x = min(0.06, 0.5 * remaining)
        else:
            self.finished = True
            self.get_logger().info(
                f"Reached target after {distance:.2f} m"
            )

        self.publisher.publish(cmd)


def main():
    rclpy.init()
    node = MoveToIntersection()

    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            stop = TwistStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            node.publisher.publish(stop)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
