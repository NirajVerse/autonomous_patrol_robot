import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TwistStamped
from route_planner import nodes, find_route


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, limit):
    return max(-limit, min(limit, value))


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')
        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        self.stations = ['P1', 'P2', 'P3']
        self.station_index = 0
        self.current_node = 'START'
        self.blocked_roads = set()
        self.route = []
        self.segment_index = 0
        self.phase = 'PLAN'  # PLAN, ROTATE, DRIVE, RECOVER_ROTATE, RECOVER_DRIVE
        self.finished = False
        self.report = {}

        self.world_start = (0.0, -1.3, math.pi / 2)
        self.initial_odom = None
        self.pose = None
        self.scan_ready = False
        self.front_distance = float('inf')
        self.closest_distance = float('inf')
        self.recovery_from = None
        self.recovery_to = None
        self.recovery_start = None

        self.k_heading = 1.5
        self.k_lateral = 2.0
        self.max_linear = 0.04
        self.max_angular = 0.20
        self.arrival_tolerance = 0.015
        self.max_lateral_error = 0.06
        self.timer = self.create_timer(0.1, self.control)
        self.get_logger().info('Patrol manager with recovery ready')

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        self.pose = (p.x, p.y, yaw)
        if self.initial_odom is None:
            self.initial_odom = self.pose
            self.get_logger().info('Initial odometry recorded')

    def on_scan(self, msg):
        front = []
        all_ranges = []
        for i, r in enumerate(msg.ranges):
            if not (math.isfinite(r) and msg.range_min <= r <= msg.range_max):
                continue
            all_ranges.append(r)
            a = wrap(msg.angle_min + i * msg.angle_increment)
            if abs(a) < math.radians(20):
                front.append(r)
        self.front_distance = min(front, default=float('inf'))
        self.closest_distance = min(all_ranges, default=float('inf'))
        self.scan_ready = True

    def command(self, linear=0.0, angular=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self.cmd_pub.publish(msg)

    def world_to_odom(self, wx, wy):
        ox, oy, oyaw = self.initial_odom
        sx, sy, syaw = self.world_start
        dx, dy = wx-sx, wy-sy
        rot = wrap(oyaw - syaw)
        return (ox + dx*math.cos(rot) - dy*math.sin(rot),
                oy + dx*math.sin(rot) + dy*math.cos(rot))

    def halt(self, reason):
        self.command()
        self.finished = True
        self.get_logger().warning(reason)
        self.print_report()

    def print_report(self):
        self.get_logger().info('===== PATROL REPORT =====')
        for station in self.stations:
            self.get_logger().info(
                f'{station}: {self.report.get(station, "NOT VISITED")}')
        self.get_logger().info(f'Blocked roads: {len(self.blocked_roads)}')
        self.get_logger().info('=========================')

    def plan(self):
        if self.station_index >= len(self.stations):
            self.command()
            self.finished = True
            self.get_logger().info('Patrol completed!')
            self.print_report()
            return
        goal = self.stations[self.station_index]
        path = find_route(self.current_node, goal, self.blocked_roads)
        if not path:
            self.halt(f'No available route from {self.current_node} to {goal}')
            return
        self.route = path
        self.segment_index = 0
        self.phase = 'ROTATE'
        self.get_logger().info('Route to %s: %s' % (goal, ' -> '.join(path)))

    def finish_segment(self, end_name, lateral):
        self.command()
        self.current_node = end_name
        self.get_logger().info(f'Reached {end_name}, lateral={lateral:.3f} m')
        self.segment_index += 1
        self.phase = 'ROTATE'
        if self.segment_index < len(self.route)-1:
            return
        station = self.stations[self.station_index]
        self.report[station] = 'VISITED (odometry estimate)'
        self.get_logger().info(f'{station}: VISITED')
        self.station_index += 1
        self.route = []
        self.phase = 'PLAN'

    def begin_recovery(self, start_name, end_name):
        self.command()
        self.blocked_roads.add(frozenset((start_name, end_name)))
        self.get_logger().warning(
            f'Blocked {start_name} <-> {end_name}; returning to {start_name}')
        self.recovery_from = start_name
        self.recovery_to = end_name
        self.recovery_start = (self.pose[0], self.pose[1])
        self.phase = 'RECOVER_ROTATE'

    def follow_line(self, start, end, recovery=False):
        x, y, yaw = self.pose
        sx, sy = start
        ex, ey = end
        lx, ly = ex-sx, ey-sy
        length = math.hypot(lx, ly)
        if length < 1e-5:
            self.halt('Invalid zero-length segment')
            return None
        ux, uy = lx/length, ly/length
        dx, dy = x-sx, y-sy
        lateral = ux*dy - uy*dx
        remaining = length - (dx*ux + dy*uy)
        distance_to_end = math.hypot(ex-x, ey-y)
        road_yaw = math.atan2(ly, lx)

        if abs(lateral) > self.max_lateral_error:
            self.halt(f'Excessive estimated lateral deviation: {lateral:.3f} m')
            return None

        if remaining <= self.arrival_tolerance and distance_to_end <= 0.055:
            self.command()
            return 'REACHED', lateral

        # Stop if the direction of travel is blocked.
        if self.front_distance < 0.28:
            self.halt('Obstacle ahead while returning' if recovery
                      else 'Obstacle ahead while driving')
            return None

        desired_yaw = road_yaw - math.atan(self.k_lateral*lateral)
        error = wrap(desired_yaw - yaw)
        angular = clamp(self.k_heading*error, self.max_angular)
        linear = min(self.max_linear, max(0.01, 0.5*max(0, remaining)))
        if abs(error) > 0.15:
            linear = 0.0
        self.command(linear, angular)
        return 'DRIVING', lateral

    def control(self):
        if self.finished or self.pose is None or not self.scan_ready:
            return
        if self.phase == 'PLAN':
            self.plan()
            return

        if self.phase.startswith('RECOVER'):
            back = self.world_to_odom(*nodes[self.recovery_from])
            x, y, yaw = self.pose
            desired = math.atan2(back[1]-y, back[0]-x)
            if self.phase == 'RECOVER_ROTATE':
                # Simplified safety check, not a full footprint clearance test.
                if self.closest_distance < 0.18:
                    self.halt('Too close to obstacle for recovery turn')
                    return
                error = wrap(desired-yaw)
                if abs(error) < 0.015:
                    self.command()
                    self.phase = 'RECOVER_DRIVE'
                    self.get_logger().info('Turned around. Returning to intersection.')
                else:
                    self.command(angular=clamp(1.2*error, self.max_angular))
                return

            result = self.follow_line(self.recovery_start, back, recovery=True)
            if result is not None and result[0] == 'REACHED':
                self.get_logger().info(f'Returned to {self.recovery_from}')
                self.current_node = self.recovery_from
                self.route = []
                self.phase = 'PLAN'
            return

        start_name = self.route[self.segment_index]
        end_name = self.route[self.segment_index+1]
        start = self.world_to_odom(*nodes[start_name])
        end = self.world_to_odom(*nodes[end_name])
        x, y, yaw = self.pose
        if self.phase == 'ROTATE':
            # Stop if space is too limited to rotate in place.
            if self.closest_distance < 0.18:
                self.halt('Too close to an obstacle for turning')
                return
            road_yaw = math.atan2(end[1]-start[1], end[0]-start[0])
            error = wrap(road_yaw-yaw)
            if abs(error) < 0.015:
                self.command()
                self.phase = 'DRIVE'
                self.get_logger().info(f'Aligned: {start_name} -> {end_name}')
            else:
                self.command(angular=clamp(self.k_heading*error, self.max_angular))
            return

        # Only during normal patrol driving do we close an edge and recover.
        if self.front_distance < 0.28:
            self.begin_recovery(start_name, end_name)
            return
        result = self.follow_line(start, end)
        if result is not None and result[0] == 'REACHED':
            self.finish_segment(end_name, result[1])


def main():
    rclpy.init()
    node = PatrolManager()
    try:
        while rclpy.ok() and not node.finished:
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


if __name__ == '__main__':
    main()
