import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        distances = [
            r for r in msg.ranges
            if math.isfinite(r)
            and msg.range_min <= r <= msg.range_max
        ]

        if distances:
            nearest = min(distances)
            self.get_logger().info(
                f"Nearest obstacle: {nearest:.2f} meters"
            )

def main():
    rclpy.init()
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
