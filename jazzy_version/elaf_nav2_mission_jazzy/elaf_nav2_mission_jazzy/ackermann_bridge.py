"""Twist -> Ackermann, with limits, watchdogs, and a buffered costmap scan."""
import math
import time
from collections import deque

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster


class AckermannBridge(Node):
    def __init__(self):
        super().__init__('ackermann_bridge')
        defaults = dict(wheelbase=0.3302, max_steering=0.4, max_speed=1.0,
                        acceleration=1.0, deceleration=2.0, steering_rate=3.2, max_reverse_speed=0.2,
                        command_timeout=0.4, sensor_timeout=0.5)
        self.p = {key: self.declare_parameter(key, value).value
                  for key, value in defaults.items()}
        if any(not math.isfinite(v) or v <= 0 for v in self.p.values()):
            raise ValueError('Bridge limits must be positive and finite')
        # The simulator publishes its ST center-of-gravity pose as base_link.
        # Ackermann's nonholonomic reference belongs at the rear axle (lr behind CoG).
        self.rear_tf = StaticTransformBroadcaster(self)
        rear = TransformStamped()
        rear.header.stamp = self.get_clock().now().to_msg()
        rear.header.frame_id = 'ego_racecar/base_link'
        rear.child_frame_id = 'ego_racecar/rear_axle'
        rear.transform.translation.x = -0.17145
        rear.transform.rotation.w = 1.0
        self.rear_tf.sendTransform(rear)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 1)
        self.scan_pub = self.create_publisher(LaserScan, '/elaf/scan', qos_profile_sensor_data)
        self.create_subscription(Twist, '/elaf/cmd_vel', self.command, 1)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/ego_racecar/collision', self.collision, 1)
        self.pending_scans = deque(maxlen=100)
        self.v = self.w = self.speed = self.steering = 0.0
        self.last_command = self.last_scan = self.last_odom = -math.inf
        self.contact_stop = False
        self.conflict = False
        self.last_graph = 0.0
        self.last_tick = time.monotonic()
        wall = Clock(clock_type=ClockType.STEADY_TIME)
        self.control_timer = self.create_timer(0.02, self.tick, clock=wall)
        self.create_timer(0.05, self.relay_scan, clock=wall)
        self.get_logger().info(f'Ackermann interface ready; speed cap {self.p["max_speed"]:.2f} m/s')

    def command(self, msg):
        if math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z):
            self.v, self.w = msg.linear.x, msg.angular.z
            self.last_command = time.monotonic()
        else:
            self.last_command = -math.inf

    def scan(self, msg):
        if msg.header.frame_id != 'ego_racecar/laser' or not msg.ranges:
            return
        self.last_scan = time.monotonic()
        self.pending_scans.append((self.last_scan, msg))

    def relay_scan(self):
        # Preserve timestamps; Humble's LaserScan filter waits for future TF.
        candidate = None
        while self.pending_scans and time.monotonic() - self.pending_scans[0][0] >= 0.1:
            _, candidate = self.pending_scans.popleft()
        if candidate is not None:
            self.scan_pub.publish(candidate)

    def odom(self, msg):
        if msg.child_frame_id == 'ego_racecar/base_link':
            self.last_odom = time.monotonic()

    def collision(self, msg):
        if msg.data and not self.contact_stop:
            self.contact_stop = True
            self.get_logger().error('Collision: driving stopped. Restart after resolving contact.')

    def publish(self, speed, steering):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ego_racecar/rear_axle'
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering)
        self.drive_pub.publish(msg)

    def tick(self):
        now = time.monotonic()
        dt = min(now - self.last_tick, 0.05)
        self.last_tick = now
        if now - self.last_graph > 1.0:
            names = [p.node_name for p in self.get_publishers_info_by_topic('/elaf/cmd_vel')]
            nav2_pair = sorted(names) == ['behavior_server', 'controller_server']
            conflict = self.count_publishers('/drive') > 1 or (len(names) > 1 and not nav2_pair)
            if conflict and not self.conflict:
                self.get_logger().error('Multiple driving publishers detected; stop the other controller.')
            self.conflict = conflict
            self.last_graph = now
        stale = (now - self.last_command > self.p['command_timeout'] or
                 now - min(self.last_scan, self.last_odom) > self.p['sensor_timeout'])
        if stale or self.contact_stop or self.conflict or abs(self.v) <= 0.005:
            self.speed = 0.0
            self.publish(0.0, 0.0)
            return
        steering = math.atan(self.p['wheelbase'] * self.w / self.v)
        steering = max(-self.p['max_steering'], min(self.p['max_steering'], steering))
        target = max(-self.p['max_reverse_speed'], min(self.v, self.p['max_speed']))
        rate = self.p['acceleration'] if abs(target) > abs(self.speed) else self.p['deceleration']
        self.speed += max(-rate * dt, min(rate * dt, target - self.speed))
        self.steering += max(-self.p['steering_rate'] * dt,
                             min(self.p['steering_rate'] * dt, steering - self.steering))
        self.publish(self.speed, self.steering)


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = AckermannBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for _ in range(5):
                node.publish(0.0, 0.0)
                time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
