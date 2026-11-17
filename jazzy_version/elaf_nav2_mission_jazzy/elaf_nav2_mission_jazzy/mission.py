"""Four ordered Nav2 goals, automatic readiness, and three laps."""
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool


class Mission(Node):
    def __init__(self):
        super().__init__('mission')
        self.goals = [self.declare_parameter(f'waypoint_{i}', [0.0, 0.0, 0.0]).value
                      for i in range(1, 5)]
        self.laps = self.declare_parameter('laps', 3).value
        self.switch_distance = self.declare_parameter('switch_distance', 1.5).value
        self.start_x = self.declare_parameter('start_line_x', -12.0).value
        self.timeout = self.declare_parameter('timeout', 1200.0).value
        if self.laps < 1 or any(len(p) != 3 or not all(math.isfinite(v) for v in p)
                               for p in self.goals):
            raise ValueError('Provide four [x, y, yaw] goals and laps >= 1')
        self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.ready_client = self.create_client(GetState, '/bt_navigator/get_state')
        self.ready_future = None
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/ego_racecar/collision', self.collision, 1)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.position = None
        self.map = None
        self.goal_handle = None
        self.index = 0
        self.completed = 0
        self.generation = 0
        self.state = 'waiting'
        self.success = False
        self.created = time.monotonic()
        self.lap_start = None
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'Waiting for Nav2; mission requests {self.laps} laps')

    def odom(self, msg):
        p = msg.pose.pose
        q = p.orientation
        yaw = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)
        self.position = (p.position.x - 0.17145*math.cos(yaw),
                         p.position.y - 0.17145*math.sin(yaw))

    def map_callback(self, msg):
        self.map = msg

    def collision(self, msg):
        if msg.data:
            self.stop('Collision detected; mission cancelled')

    def stop(self, message, success=False):
        if self.state == 'done':
            return
        self.state = 'done'
        self.success = success
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        (self.get_logger().info if success else self.get_logger().error)(message)

    def validate_goals(self):
        """Check filled physical footprints, not just the goal center cell."""
        msg = self.map
        data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        res = msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        u, v = np.meshgrid(np.arange(-0.1349, 0.4652, res / 2),
                           np.arange(-0.165, 0.166, res / 2))
        for i, (x, y, yaw) in enumerate(self.goals):
            cols = np.floor((x + np.cos(yaw)*u - np.sin(yaw)*v - ox) / res).astype(int)
            rows = np.floor((y + np.sin(yaw)*u + np.cos(yaw)*v - oy) / res).astype(int)
            if (np.any(cols < 0) or np.any(cols >= msg.info.width) or
                    np.any(rows < 0) or np.any(rows >= msg.info.height) or
                    np.any(data[rows, cols] != 0)):
                self.stop(f'Waypoint {i + 1} has an occupied or unknown footprint; edit the YAML')
                return False
        return True

    def tick(self):
        if self.state == 'done':
            return
        if time.monotonic() - self.created > self.timeout:
            self.stop('Mission timeout')
            return
        if self.state == 'waiting':
            if time.monotonic() - self.created > 90.0:
                self.stop('Nav2 startup timed out')
                return
            if self.map is None or self.position is None or not self.client.server_is_ready():
                return
            if self.ready_future is None:
                if self.ready_client.service_is_ready():
                    self.ready_future = self.ready_client.call_async(GetState.Request())
                return
            if not self.ready_future.done():
                return
            active = self.ready_future.result().current_state.id == 3
            self.ready_future = None
            if active and self.validate_goals():
                self.lap_start = time.monotonic()
                self.send_goal()
            return
        if self.state != 'driving' or self.position is None:
            return
        distance = math.dist(self.position, self.goals[self.index][:2])
        if self.index < 3 and distance < self.switch_distance:
            self.index += 1
            self.send_goal()
        elif (self.index == 3 and self.completed < self.laps - 1 and
              self.position[0] >= self.start_x and abs(self.position[1]) < 0.6):
            # The fourth pose is beyond the start line, avoiding an early lap count.
            self.finish_lap()
            self.index = 0
            self.send_goal()

    def send_goal(self):
        self.generation += 1
        generation = self.generation
        self.state = 'sending'
        x, y, yaw = self.goals[self.index]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z = math.sin(yaw / 2)
        pose.pose.orientation.w = math.cos(yaw / 2)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(f'Lap {self.completed + 1}/{self.laps}: waypoint {self.index + 1}')
        self.client.send_goal_async(goal).add_done_callback(lambda f: self.accepted(f, generation))

    def accepted(self, future, generation):
        handle = future.result()
        if not handle.accepted:
            self.stop('Nav2 rejected the goal')
            return
        self.goal_handle = handle
        if self.state == 'done':
            handle.cancel_goal_async()
            return
        self.state = 'driving'
        handle.get_result_async().add_done_callback(lambda f: self.result(f, generation))

    def finish_lap(self):
        self.completed += 1
        now = time.monotonic()
        self.get_logger().info(f'Completed lap {self.completed}/{self.laps} in {now - self.lap_start:.1f} s')
        self.lap_start = now

    def result(self, future, generation):
        if self.state == 'done' or generation != self.generation:
            return  # Previous goal was deliberately replaced to avoid a full stop.
        if future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.stop(f'Navigation failed at waypoint {self.index + 1}')
        elif self.index < 3:
            self.index += 1
            self.send_goal()
        else:
            self.finish_lap()
            if self.completed >= self.laps:
                self.stop('Mission complete: three-lap target reached' if self.laps == 3
                          else 'Mission complete', success=True)
            else:
                self.index = 0
                self.send_goal()


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = Mission()
    try:
        while rclpy.ok() and node.state != 'done':
            rclpy.spin_once(node, timeout_sec=0.1)
        end = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.stop('Mission interrupted')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not node.success:
        raise SystemExit(1)
