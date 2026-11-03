#!/usr/bin/env python3

import heapq
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf2_ros import Buffer, TransformException, TransformListener

from leader_follower.formation_manager import FormationManager, FormationState
from leader_follower.obstacle_avoidance import ObstacleAvoidance


class SavedMap:
    def __init__(self, yaml_path: str, inflation_radius: float):
        self.yaml_path = Path(yaml_path).expanduser() if yaml_path else None
        self.inflation_radius = inflation_radius
        self.loaded = False
        self.width = 0
        self.height = 0
        self.resolution = 0.05
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.cells = []
        self.inflated = []

        if self.yaml_path:
            self.load()

    def load(self):
        if not self.yaml_path or not self.yaml_path.exists():
            return

        values = {}
        with self.yaml_path.open('r', encoding='utf-8') as file:
            for raw in file:
                line = raw.strip()
                if not line or line.startswith('#') or ':' not in line:
                    continue
                key, value = line.split(':', 1)
                values[key.strip()] = value.strip()

        image_name = values.get('image', '').strip().strip('"\'')
        image_path = (self.yaml_path.parent / image_name).resolve()

        origin_raw = values.get('origin', '[0, 0, 0]')
        origin_raw = origin_raw.strip('[] ')
        origin = [float(v.strip()) for v in origin_raw.split(',')]

        self.resolution = float(values.get('resolution', '0.05'))
        self.origin_x = origin[0]
        self.origin_y = origin[1]
        occupied_thresh = float(values.get('occupied_thresh', '0.65'))
        negate = int(values.get('negate', '0'))

        width, height, pixels = self._read_pgm(image_path)
        self.width = width
        self.height = height
        self.cells = [0] * (width * height)

        # PGM image rows start at the top of the image, while ROS map
        # coordinates start at the bottom. Flip the row index so the
        # planner uses the same coordinate convention as the saved ROS map.
        for row in range(height):
            ros_y = height - 1 - row
            for x in range(width):
                pixel = pixels[row * width + x]
                if negate:
                    probability = pixel / 255.0
                else:
                    probability = (255.0 - pixel) / 255.0
                self.cells[ros_y * width + x] = (
                    1 if probability >= occupied_thresh else 0
                )

        self._inflate()
        self.loaded = True

    @staticmethod
    def _read_pgm(path: Path):
        with path.open('rb') as file:
            if file.readline().strip() != b'P5':
                raise ValueError('Only P5 PGM maps are supported.')

            tokens = []
            while len(tokens) < 3:
                line = file.readline()
                if not line:
                    raise ValueError('Invalid PGM header.')
                line = line.strip()
                if not line or line.startswith(b'#'):
                    continue
                tokens.extend(line.split())

            width = int(tokens[0])
            height = int(tokens[1])
            max_value = int(tokens[2])
            if max_value > 255:
                raise ValueError('16-bit PGM is not supported.')

            data = list(file.read(width * height))
            if len(data) != width * height:
                raise ValueError('Incomplete PGM image.')
            return width, height, data

    def _inflate(self):
        radius = max(1, int(math.ceil(self.inflation_radius / self.resolution)))
        self.inflated = list(self.cells)
        occupied = []

        for y in range(self.height):
            for x in range(self.width):
                if self.cells[y * self.width + x]:
                    occupied.append((x, y))

        for ox, oy in occupied:
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue
                    nx = ox + dx
                    ny = oy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        self.inflated[ny * self.width + nx] = 1

    def world_to_grid(self, x, y):
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def grid_to_world(self, x, y):
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )

    def inside(self, x, y):
        return (
            0 <= x < self.width
            and 0 <= y < self.height
        )

    def free(self, x, y):
        return (
            0 <= x < self.width
            and 0 <= y < self.height
            and self.inflated[y * self.width + x] == 0
        )

    def nearest_free(self, grid, radius=12):
        gx, gy = grid
        if self.free(gx, gy):
            return grid

        for r in range(1, radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue
                    nx, ny = gx + dx, gy + dy
                    if self.free(nx, ny):
                        return nx, ny
        return None

    def plan(self, start_world, goal_world):
        if not self.loaded:
            return []

        start = self.nearest_free(self.world_to_grid(*start_world))
        goal = self.nearest_free(self.world_to_grid(*goal_world))
        if start is None or goal is None:
            return []
        if start == goal:
            return [self.grid_to_world(*start)]

        open_heap = [(0.0, 0, start)]
        came_from = {}
        g_cost = {start: 0.0}
        visited = set()
        counter = 0

        neighbours = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        while open_heap:
            _, _, current = heapq.heappop(open_heap)
            if current in visited:
                continue
            visited.add(current)

            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return [self.grid_to_world(*p) for p in path]

            cx, cy = current
            for dx, dy in neighbours:
                nx, ny = cx + dx, cy + dy
                if not self.free(nx, ny):
                    continue
                if dx and dy and (
                    not self.free(cx + dx, cy)
                    or not self.free(cx, cy + dy)
                ):
                    continue

                neighbour = (nx, ny)
                step = math.sqrt(2.0) if dx and dy else 1.0
                new_cost = g_cost[current] + step

                if neighbour not in g_cost or new_cost < g_cost[neighbour]:
                    g_cost[neighbour] = new_cost
                    heuristic = math.hypot(goal[0] - nx, goal[1] - ny)
                    came_from[neighbour] = current
                    counter += 1
                    heapq.heappush(
                        open_heap,
                        (new_cost + heuristic, counter, neighbour),
                    )

        return []


class FollowerController(Node):
    def __init__(self):
        super().__init__('follower_controller')

        self.declare_parameter('desired_distance', 1.0)
        self.declare_parameter('spacing', 1.0)
        self.declare_parameter('follower_number', 1)
        self.declare_parameter('slot_tolerance', 0.08)
        self.declare_parameter('rejoin_distance', 1.20)
        self.declare_parameter('linear_gain', 0.8)
        self.declare_parameter('angular_gain', 1.8)
        self.declare_parameter('max_linear_speed', 0.60)
        self.declare_parameter('max_angular_speed', 1.10)
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('data_timeout', 0.50)
        self.declare_parameter('stuck_timeout', 1.2)
        self.declare_parameter('stuck_min_command', 0.08)
        self.declare_parameter('stuck_min_motion', 0.02)
        self.declare_parameter('recovery_turn_time', 1.0)
        self.declare_parameter('base_frame', 'follower_base_link')
        self.declare_parameter('leader_frame', 'leader_base_link')
        self.declare_parameter('reference_frame', '')
        self.declare_parameter('odom_topic', '/follower/odom')
        self.declare_parameter('scan_topic', '/follower/front_lidar/scan')
        self.declare_parameter('cmd_vel_topic', '/follower/cmd_vel')
        self.declare_parameter('map_yaml', '')
        self.declare_parameter('map_inflation_radius', 0.35)
        self.declare_parameter('map_replan_distance', 0.15)
        self.declare_parameter('dynamic_replan_interval', 0.25)
        self.declare_parameter('dynamic_obstacle_range', 6.0)
        self.declare_parameter('dynamic_inflation_radius', 0.40)
        self.declare_parameter('obstacle_stop_distance', 0.48)
        self.declare_parameter('obstacle_slow_distance', 0.90)
        self.declare_parameter('local_inflation_radius', 0.40)
        self.declare_parameter('orientation_tolerance', 0.08)
        self.declare_parameter('orientation_gain', 1.5)
        self.declare_parameter('formation_safety_radius', 0.65)
        self.declare_parameter('safe_slot_search_radius', 1.50)
        self.declare_parameter('initialpose_topic', '')

        spacing = float(self.get_parameter('spacing').value)
        desired_distance = float(self.get_parameter('desired_distance').value)
        follower_number = int(self.get_parameter('follower_number').value)

        self.follower_number = follower_number

        self.formation = FormationManager(
            spacing=spacing,
            slot_tolerance=float(self.get_parameter('slot_tolerance').value),
            rejoin_distance=float(self.get_parameter('rejoin_distance').value),
            max_catchup_speed=float(self.get_parameter('max_linear_speed').value),
        )
        self.formation.set_follower_number(follower_number)

        # Explicit desired_distance remains supported, but the numbered
        # slot is authoritative when follower_number is supplied.
        self.formation.spacing = desired_distance / max(follower_number, 1)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.leader_frame = str(self.get_parameter('leader_frame').value)
        configured_reference = str(self.get_parameter('reference_frame').value).strip()
        self.reference_frame = (
            configured_reference
            if configured_reference
            else (
                self.leader_frame
                if self.follower_number == 1
                else f'follower_{self.follower_number - 1}_base_link'
            )
        )
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)

        self.linear_gain = float(self.get_parameter('linear_gain').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.data_timeout = float(self.get_parameter('data_timeout').value)

        self.stuck_timeout = float(self.get_parameter('stuck_timeout').value)
        self.stuck_min_command = float(self.get_parameter('stuck_min_command').value)
        self.stuck_min_motion = float(self.get_parameter('stuck_min_motion').value)
        self.recovery_turn_time = float(self.get_parameter('recovery_turn_time').value)

        self.obstacle = ObstacleAvoidance(
            stop_distance=float(self.get_parameter('obstacle_stop_distance').value),
            slow_distance=float(self.get_parameter('obstacle_slow_distance').value),
            inflation_radius=float(self.get_parameter('local_inflation_radius').value),
        )

        self.map = SavedMap(
            str(self.get_parameter('map_yaml').value),
            float(self.get_parameter('map_inflation_radius').value),
        )

        self.map_replan_distance = float(
            self.get_parameter('map_replan_distance').value
        )
        self.dynamic_replan_interval = float(
            self.get_parameter('dynamic_replan_interval').value
        )
        self.dynamic_obstacle_range = float(
            self.get_parameter('dynamic_obstacle_range').value
        )
        self.dynamic_inflation_radius = float(
            self.get_parameter('dynamic_inflation_radius').value
        )

        self.orientation_tolerance = float(
            self.get_parameter('orientation_tolerance').value
        )
        self.orientation_gain = float(
            self.get_parameter('orientation_gain').value
        )

        configured_initialpose_topic = str(
            self.get_parameter('initialpose_topic').value
        ).strip()
        if configured_initialpose_topic:
            self.initialpose_topic = configured_initialpose_topic
        else:
            self.initialpose_topic = (
                f'/follower_{self.follower_number}/amcl_initialpose'
            )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_scan_time = None
        self.last_odom_time = None
        self.last_odom_x = None
        self.last_odom_y = None
        self.stuck_since = None
        self.recovery_since = None
        self.no_path_since = None

        # Latest scan used by the dynamic A* layer.
        self.last_scan = None
        self.last_dynamic_update_time = 0.0
        self.dynamic_obstacles = set()
        self.dynamic_obstacles_inflated = set()

        self.auto_initial_pose_sent = False
        self.last_auto_initial_pose_time = 0.0

        self.path = []
        self.path_index = 0
        self.last_goal_map = None
        self.last_command = Twist()

        self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, 10
        )
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, 10
        )
        self.publisher = self.create_publisher(
            Twist, self.cmd_vel_topic, 10
        )

        self.initialpose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            self.initialpose_topic,
            10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10,
        )

        rate = float(self.get_parameter('control_rate').value)
        self.create_timer(1.0 / rate, self.control_loop)
        self.create_timer(0.5, self.debug_status)

        self.get_logger().info(
            f'Follower {follower_number}: follows {self.reference_frame} '
            f'at {float(self.get_parameter('spacing').value):.2f} m spacing'
        )
        self.get_logger().info(
            f'Frames: {self.base_frame} <- {self.reference_frame}'
        )
        self.get_logger().info(
            f'Topics: {self.scan_topic}, {self.odom_topic}, '
            f'{self.cmd_vel_topic}'
        )
        self.get_logger().info(
            f'Saved map loaded: {self.map.loaded}'
        )

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        self.last_scan = msg
        self.obstacle.update_scan(msg)

    def odom_callback(self, msg):
        now = self.get_clock().now()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.last_odom_time is None:
            self.last_odom_time = now
            self.last_odom_x = x
            self.last_odom_y = y
            return

        dt = (now - self.last_odom_time).nanoseconds / 1e9
        if dt < 0.10:
            return

        motion = math.hypot(
            x - self.last_odom_x,
            y - self.last_odom_y,
        )

        if (
            self.last_command.linear.x >= self.stuck_min_command
            and motion < self.stuck_min_motion
        ):
            if self.stuck_since is None:
                self.stuck_since = now
        else:
            self.stuck_since = None

        self.last_odom_time = now
        self.last_odom_x = x
        self.last_odom_y = y

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def transform_point(transform, x, y):
        q = transform.transform.rotation

        tx = 2.0 * (q.y * 0.0 - q.z * y)
        ty = 2.0 * (q.z * x - q.x * 0.0)
        tz = 2.0 * (q.x * y - q.y * x)

        rx = x + q.w * tx + (q.y * tz - q.z * ty)
        ry = y + q.w * ty + (q.z * tx - q.x * tz)

        return (
            rx + transform.transform.translation.x,
            ry + transform.transform.translation.y,
        )

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_follower_initial_pose(self, leader_x, leader_y, leader_yaw, covariance=None):
        distance = self.formation.slot_distance()

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = (
            leader_x - distance * math.cos(leader_yaw)
        )
        msg.pose.pose.position.y = (
            leader_y - distance * math.sin(leader_yaw)
        )
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.z = math.sin(leader_yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(leader_yaw / 2.0)

        if covariance is not None and len(covariance) == 36:
            msg.pose.covariance = list(covariance)
        else:
            msg.pose.covariance[0] = 0.25
            msg.pose.covariance[7] = 0.25
            msg.pose.covariance[35] = 0.10

        self.initialpose_publisher.publish(msg)

    def initialpose_callback(self, msg):
        if msg.header.frame_id not in ('map', '/map'):
            return

        q = msg.pose.pose.orientation
        leader_yaw = self.yaw_from_quaternion(q)
        leader_x = msg.pose.pose.position.x
        leader_y = msg.pose.pose.position.y

        self.publish_follower_initial_pose(
            leader_x,
            leader_y,
            leader_yaw,
            msg.pose.covariance,
        )

        self.auto_initial_pose_sent = True
        self.last_auto_initial_pose_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        self.get_logger().info(
            f'Follower {self.follower_number}: received leader 2D pose estimate; '
            f'initialized AMCL at its {self.formation.slot_distance():.2f} m absolute slot.'
        )

    def auto_initialize_follower_amcl(self, leader_map):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # Initialize follower AMCL only once automatically.  After AMCL has
        # been seeded, never reset it just because TF is temporarily
        # unavailable; a second automatic reset can create map/Gazebo drift.
        if self.auto_initial_pose_sent:
            return

        leader_x = leader_map.transform.translation.x
        leader_y = leader_map.transform.translation.y
        leader_yaw = self.yaw_from_quaternion(
            leader_map.transform.rotation
        )

        self.publish_follower_initial_pose(
            leader_x,
            leader_y,
            leader_yaw,
        )

        self.auto_initial_pose_sent = True
        self.last_auto_initial_pose_time = now_sec

        self.get_logger().info(
            f'Follower {self.follower_number}: auto-initialized AMCL from leader pose.'
        )

    def align_orientation(self, leader_tf, follower_tf):
        leader_yaw = self.yaw_from_quaternion(
            leader_tf.transform.rotation
        )
        follower_yaw = self.yaw_from_quaternion(
            follower_tf.transform.rotation
        )

        error = self.normalize_angle(leader_yaw - follower_yaw)

        if abs(error) <= self.orientation_tolerance:
            return None

        cmd = Twist()
        cmd.angular.z = max(
            -self.max_angular_speed,
            min(
                self.max_angular_speed,
                self.orientation_gain * error,
            ),
        )
        return cmd

    def get_reference_map_tf(self):
        """Return the robot this follower should follow in the map frame.

        Follower 1 follows the leader. Each later follower follows the
        immediately preceding follower, keeping the formation as a chain.
        """
        return self._lookup('map', self.reference_frame)

    def get_formation_slot_map(self):
        """Compute this follower's one-spacing slot behind its reference robot."""
        reference_map = self.get_reference_map_tf()
        if reference_map is None:
            return None

        rx = reference_map.transform.translation.x
        ry = reference_map.transform.translation.y
        yaw = self.yaw_from_quaternion(reference_map.transform.rotation)
        spacing = float(self.get_parameter('spacing').value)

        return (
            rx - spacing * math.cos(yaw),
            ry - spacing * math.sin(yaw),
        )

    def get_direct_slot(self):
        """Return the formation slot in the follower base frame."""
        transform = self._lookup(
            self.base_frame,
            self.leader_frame,
        )

        if transform is None:
            return None

        leader_x = transform.transform.translation.x
        leader_y = transform.transform.translation.y

        q = transform.transform.rotation
        leader_forward_x = (
            1.0
            - 2.0 * (q.y * q.y + q.z * q.z)
        )
        leader_forward_y = (
            2.0 * (q.x * q.y + q.w * q.z)
        )

        distance = self.formation.slot_distance()

        return (
            leader_x - distance * leader_forward_x,
            leader_y - distance * leader_forward_y,
        )

    def _lookup(self, target, source):
        try:
            return self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

    def update_dynamic_obstacles(self, force=False):
        """Build a temporary live-obstacle layer from the latest LiDAR scan."""
        if not self.map.loaded or self.last_scan is None:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if (
            not force
            and now_sec - self.last_dynamic_update_time
            < self.dynamic_replan_interval
        ):
            return

        scan = self.last_scan

        scan_to_map = self._lookup(
            'map',
            scan.header.frame_id
        )

        # Keep the current layer until a valid map transform is available.
        if scan_to_map is None:
            return

        obstacles = set()
        angle = scan.angle_min
        max_range = min(
            scan.range_max,
            self.dynamic_obstacle_range
        )

        for distance in scan.ranges:
            if not math.isfinite(distance):
                angle += scan.angle_increment
                continue

            if distance < scan.range_min or distance > max_range:
                angle += scan.angle_increment
                continue

            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)

            map_x, map_y = self.transform_point(
                scan_to_map,
                local_x,
                local_y
            )

            gx, gy = self.map.world_to_grid(
                map_x,
                map_y
            )

            # Saved-map walls are already handled by SavedMap.
            # Only add newly observed points in statically free cells.
            if self.map.inside(gx, gy) and self.map.free(gx, gy):
                obstacles.add((gx, gy))

            angle += scan.angle_increment

        self.dynamic_obstacles = obstacles
        self.dynamic_obstacles_inflated = (
            self.inflate_dynamic_obstacles(obstacles)
        )
        self.last_dynamic_update_time = now_sec

    def inflate_dynamic_obstacles(self, obstacles):
        """Inflate live obstacles so A* keeps the robot's clearance."""
        if not obstacles:
            return set()

        radius = max(
            1,
            int(
                math.ceil(
                    self.dynamic_inflation_radius
                    / self.map.resolution
                )
            )
        )

        inflated = set()

        for ox, oy in obstacles:
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy > radius * radius:
                        continue

                    nx = ox + dx
                    ny = oy + dy

                    if self.map.inside(nx, ny):
                        inflated.add((nx, ny))

        return inflated

    def dynamic_free(self, x, y):
        """Check both the saved map and the live LiDAR obstacle layer."""
        if not self.map.inside(x, y):
            return False

        if not self.map.free(x, y):
            return False

        if (x, y) in self.dynamic_obstacles_inflated:
            return False

        return True

    def nearest_dynamic_free(self, grid, radius=20):
        gx, gy = grid

        if self.dynamic_free(gx, gy):
            return grid

        for r in range(1, radius + 1):
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if max(abs(dx), abs(dy)) != r:
                        continue

                    nx = gx + dx
                    ny = gy + dy

                    if self.dynamic_free(nx, ny):
                        return nx, ny

        return None

    def plan_dynamic(self, start_world, goal_world):
        """A* using both the saved map and current LiDAR obstacles."""
        if not self.map.loaded:
            return []

        start = self.map.nearest_free(
            self.map.world_to_grid(*start_world)
        )
        goal = self.map.nearest_free(
            self.map.world_to_grid(*goal_world)
        )

        if start is None or goal is None:
            return []

        if not self.dynamic_free(*start):
            start = self.nearest_dynamic_free(start)

        if not self.dynamic_free(*goal):
            goal = self.nearest_dynamic_free(goal)

        if start is None or goal is None:
            return []

        if start == goal:
            return [self.map.grid_to_world(*start)]

        open_heap = [(0.0, 0, start)]
        came_from = {}
        g_cost = {start: 0.0}
        visited = set()
        counter = 0

        neighbours = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1),
        ]

        while open_heap:
            _, _, current = heapq.heappop(open_heap)

            if current in visited:
                continue

            visited.add(current)

            if current == goal:
                path = [current]

                while current in came_from:
                    current = came_from[current]
                    path.append(current)

                path.reverse()
                return [
                    self.map.grid_to_world(*p)
                    for p in path
                ]

            cx, cy = current

            for dx, dy in neighbours:
                nx = cx + dx
                ny = cy + dy

                if not self.dynamic_free(nx, ny):
                    continue

                # Do not squeeze diagonally through a blocked corner.
                if dx and dy:
                    if not self.dynamic_free(cx + dx, cy):
                        continue
                    if not self.dynamic_free(cx, cy + dy):
                        continue

                neighbour = (nx, ny)
                step = math.sqrt(2.0) if dx and dy else 1.0
                new_cost = g_cost[current] + step

                if (
                    neighbour not in g_cost
                    or new_cost < g_cost[neighbour]
                ):
                    g_cost[neighbour] = new_cost
                    heuristic = math.hypot(
                        goal[0] - nx,
                        goal[1] - ny
                    )
                    came_from[neighbour] = current
                    counter += 1
                    heapq.heappush(
                        open_heap,
                        (
                            new_cost + heuristic,
                            counter,
                            neighbour
                        )
                    )

        return []

    def path_blocked_by_dynamic_obstacle(self):
        """Check whether live LiDAR blocks the remaining planned route."""
        if not self.path or not self.dynamic_obstacles_inflated:
            return False

        start_index = min(
            self.path_index,
            len(self.path) - 1
        )

        for px, py in self.path[start_index:]:
            gx, gy = self.map.world_to_grid(px, py)
            if (gx, gy) in self.dynamic_obstacles_inflated:
                return True

        return False

    def update_global_path(self, slot_map):
        if not self.map.loaded:
            return False

        follower_map = self._lookup('map', self.base_frame)
        if follower_map is None:
            return False

        current = (
            follower_map.transform.translation.x,
            follower_map.transform.translation.y,
        )

        self.update_dynamic_obstacles()

        goal_changed = (
            self.last_goal_map is None
            or math.hypot(
                slot_map[0] - self.last_goal_map[0],
                slot_map[1] - self.last_goal_map[1],
            ) >= self.map_replan_distance
        )
        path_blocked = self.path_blocked_by_dynamic_obstacle()

        if self.path and not goal_changed and not path_blocked:
            self.no_path_since = None
            return True

        new_path = self.plan_dynamic(current, slot_map)
        if new_path:
            self.path = new_path
            self.path_index = 0
            self.last_goal_map = slot_map
            self.no_path_since = None
            return True

        self.path = []
        self.path_index = 0
        self.last_goal_map = None
        if self.no_path_since is None:
            self.no_path_since = self.get_clock().now()
        return False

    def get_path_waypoint(self):
        if not self.path:
            return None

        follower_map = self._lookup('map', self.base_frame)
        base_from_map = self._lookup(self.base_frame, 'map')
        if follower_map is None or base_from_map is None:
            return None

        current_map = (
            follower_map.transform.translation.x,
            follower_map.transform.translation.y,
        )

        while self.path_index < len(self.path) - 1:
            px, py = self.path[self.path_index]
            if math.hypot(px - current_map[0], py - current_map[1]) < 0.18:
                self.path_index += 1
            else:
                break

        lookahead = 0.35
        chosen = self.path_index
        for index in range(self.path_index, len(self.path)):
            px, py = self.path[index]
            distance = math.hypot(px - current_map[0], py - current_map[1])
            if distance <= lookahead:
                chosen = index
            else:
                break

        px, py = self.path[chosen]
        return self.transform_point(base_from_map, px, py)

    def start_recovery_if_stuck(self):
        if self.stuck_since is None:
            return False

        age = (
            self.get_clock().now() - self.stuck_since
        ).nanoseconds / 1e9

        if age < self.stuck_timeout:
            return False

        self.recovery_since = self.get_clock().now()
        self.formation.state = FormationState.STUCK
        return True

    def recovery_command(self):
        elapsed = (
            self.get_clock().now() - self.recovery_since
        ).nanoseconds / 1e9

        direction = self.obstacle.choose_escape_direction()
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = direction * min(0.85, self.max_angular_speed)

        if elapsed >= self.recovery_turn_time:
            self.recovery_since = None
            self.stuck_since = None
            self.no_path_since = None
            self.formation.state = FormationState.REJOIN
            self.path = []
            self.path_index = 0
            self.last_goal_map = None

        return cmd

    def control_loop(self):
        now = self.get_clock().now()

        if self.last_scan_time is None:
            self.publish_stop()
            self.formation.state = FormationState.WAITING
            return

        scan_age = (now - self.last_scan_time).nanoseconds / 1e9
        if scan_age > self.data_timeout:
            self.publish_stop()
            return

        # Each robot is independently localized. The formation reference is the
        # leader for follower_1 and the preceding follower for follower_2,
        # follower_3, etc.
        leader_map_tf = self._lookup('map', self.leader_frame)
        if leader_map_tf is None:
            self.publish_stop()
            self.formation.state = FormationState.WAITING
            return

        # If the follower AMCL has not received an initial pose yet, seed it
        # from the leader's current map pose and the follower's formation slot.
        current_map_tf = self._lookup('map', self.base_frame)
        if current_map_tf is None:
            self.auto_initialize_follower_amcl(leader_map_tf)
            self.publish_stop()
            self.formation.state = FormationState.WAITING
            return

        slot_map = self.get_formation_slot_map()
        if slot_map is None:
            self.publish_stop()
            self.formation.state = FormationState.WAITING
            return


        current_map = (
            current_map_tf.transform.translation.x,
            current_map_tf.transform.translation.y,
        )
        slot_error = math.hypot(
            slot_map[0] - current_map[0],
            slot_map[1] - current_map[1],
        )

        self.formation.classify(slot_error)

        if slot_error <= self.formation.slot_tolerance:
            reference_map_tf = self.get_reference_map_tf()
            if reference_map_tf is None:
                self.publish_stop()
                self.formation.state = FormationState.WAITING
                return

            alignment_cmd = self.align_orientation(
                reference_map_tf,
                current_map_tf,
            )

            self.path = []
            self.last_goal_map = None

            if alignment_cmd is not None:
                self.publish(alignment_cmd)
                return

            self.publish_stop()
            return

        if self.recovery_since is not None:
            cmd = self.recovery_command()
            self.publish(cmd)
            return

        self.start_recovery_if_stuck()
        if self.recovery_since is not None:
            cmd = self.recovery_command()
            self.publish(cmd)
            return

        path_available = self.update_global_path(slot_map)
        waypoint = self.get_path_waypoint()

        if waypoint is None or not path_available:
            if self.no_path_since is not None:
                no_path_age = (
                    self.get_clock().now() - self.no_path_since
                ).nanoseconds / 1e9
            else:
                no_path_age = 0.0

            if no_path_age >= 0.20 and self.recovery_since is None:
                self.recovery_since = self.get_clock().now()
                self.no_path_since = None
                self.formation.state = FormationState.STUCK
                self.publish(self.recovery_command())
            else:
                self.publish_stop()
            return

        target_x, target_y = waypoint

        target_distance = math.hypot(target_x, target_y)
        target_heading = math.atan2(target_y, target_x)

        angular = self.angular_gain * target_heading
        angular = max(
            -self.max_angular_speed,
            min(self.max_angular_speed, angular),
        )

        linear = self.linear_gain * target_distance
        linear = min(self.max_linear_speed, max(0.0, linear))

        # Turn-first behaviour keeps the robot from cutting across obstacles.
        if abs(target_heading) > 0.90:
            linear = 0.0
        else:
            linear *= max(0.0, math.cos(target_heading))

        if self.formation.state == FormationState.REJOIN:
            linear = min(
                self.max_linear_speed,
                max(linear, min(0.45, self.max_linear_speed)),
            )

        linear, angular = self.obstacle.apply(linear, angular)

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        self.publish(cmd)

    def publish(self, cmd):
        self.last_command = cmd
        self.publisher.publish(cmd)

    def publish_stop(self):
        self.publish(Twist())

    def debug_status(self):
        self.get_logger().info(
            f'[{self.formation.state.value}] '
            f'spacing={float(self.get_parameter('spacing').value):.2f}m | '
            f'front={self.obstacle.front:.2f}m | '
            f'cmd=({self.last_command.linear.x:.2f}, '
            f'{self.last_command.angular.z:.2f}) | '
            f'dynamic={len(self.dynamic_obstacles)} | '
            f'path={len(self.path)}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()






