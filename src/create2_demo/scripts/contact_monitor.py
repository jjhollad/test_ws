#!/usr/bin/env python3
"""Flag chassis contacts and capture the robot state leading up to impact."""

import csv
from collections import deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path

from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


FLAG_HEADER = [
    "utc_timestamp", "ros_time_nanoseconds", "identifier", "active_run",
    "managed_processes",
]
CONTACT_HEADER = [
    "utc_timestamp", "ros_time_nanoseconds", "event", "duration_s",
    "contact_episode", "event_record",
    "behavior", "robot_collision", "environment_collision", "contact_x",
    "contact_y", "contact_z", "normal_x", "normal_y", "normal_z",
    "total_force_n", "odom_x", "odom_y", "linear_velocity",
    "angular_velocity", "command_linear", "command_angular",
    "lidar_front_m", "lidar_front_right_m", "lidar_right_m",
    "lidar_minimum_m", "lidar_age_s", "lidar_lookback_s",
]


class ContactMonitor(Node):
    """Record simulation chassis contact and its preceding navigation state."""

    def __init__(self):
        """Create contact, navigation-state, and LiDAR subscriptions."""
        super().__init__("contact_monitor")
        default_log = str(Path.home() / "test_ws" / "log")
        self.declare_parameter("log_directory", default_log)
        self.declare_parameter("contact_release_seconds", 0.15)
        self.declare_parameter("repeat_flag_seconds", 5.0)
        self.declare_parameter("pre_contact_lookback_seconds", 0.5)
        configured_log = str(self.get_parameter("log_directory").value)
        self._log_dir = Path(configured_log).expanduser()
        self._log_dir.mkdir(parents=True, exist_ok=True)
        release = self.get_parameter("contact_release_seconds").value
        repeat = self.get_parameter("repeat_flag_seconds").value
        self._release_seconds = float(release)
        self._repeat_seconds = float(repeat)
        self._lookback_seconds = float(
            self.get_parameter("pre_contact_lookback_seconds").value
        )
        self._in_contact = False
        self._contact_started_ns = 0
        self._last_contact_ns = 0
        self._last_flag_ns = -10**18
        previous = self._read_summary()
        self._contact_episode_count = int(previous.get("contact_episodes", 0))
        self._event_record_count = int(previous.get("event_records", 0))
        self._last_contact_state = None
        self._scan = None
        self._scan_ns = 0
        self._scan_history = deque(maxlen=300)
        self._odom = None
        self._command = None
        self._behavior = "unknown"

        self._contact_pub = self.create_publisher(
            Bool, "/physical_contact", 10
        )
        self._event_pub = self.create_publisher(String, "/contact_event", 10)
        self.create_subscription(
            ContactsState,
            "/chassis_contacts",
            self._contacts,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ContactsState,
            "/front_caster_contacts",
            self._contacts,
            qos_profile_sensor_data,
        )
        for topic in (
            "/left_rear_wheel_contacts", "/right_rear_wheel_contacts"
        ):
            self.create_subscription(
                ContactsState, topic, self._contacts, qos_profile_sensor_data,
            )
        self.create_subscription(
            LaserScan, "/scan", self._laser, qos_profile_sensor_data
        )
        self.create_subscription(Odometry, "/odom", self._set_odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self._set_command, 20)
        self.create_subscription(
            String, "/wall_behavior_state", self._set_behavior, 20
        )
        self.create_timer(0.05, self._check_release)
        # Create the files at startup. Their presence confirms that collision
        # logging is armed even when a run contains zero wall contacts.
        self._ensure_log_files()
        self._publish_contact(False)
        self.get_logger().info(
            f"Chassis contact tracker active; detailed events: "
            f"{self._log_dir / 'contact_events.csv'}"
        )

    def _laser(self, message):
        self._scan = message
        self._scan_ns = self.get_clock().now().nanoseconds
        self._scan_history.append((
            self._scan_ns,
            self._scan_sector(0.0, math.radians(15.0), message),
            self._scan_sector(-math.pi / 4.0, math.radians(15.0), message),
            self._scan_sector(-math.pi / 2.0, math.radians(15.0), message),
            self._finite_min(message.ranges),
        ))

    def _set_odom(self, message):
        self._odom = message

    def _set_command(self, message):
        self._command = message

    def _set_behavior(self, message):
        self._behavior = message.data.strip() or "unknown"

    def _contacts(self, message):
        states = [
            state for state in message.states
            if not self._is_routine_ground_contact(state)
        ]
        if not states:
            return
        state = states[0]
        now_ns = self.get_clock().now().nanoseconds
        self._last_contact_ns = now_ns
        self._last_contact_state = state
        new_episode = not self._in_contact
        if new_episode:
            self._in_contact = True
            self._contact_started_ns = now_ns
            self._contact_episode_count += 1
            self._publish_contact(True)
        repeat_due = (
            (now_ns - self._last_flag_ns) / 1e9 >= self._repeat_seconds
        )
        if new_episode or repeat_due:
            event = "contact_start" if new_episode else "contact_sustained"
            self._record_contact(state, now_ns, event, create_flag=True)

    @staticmethod
    def _is_routine_ground_contact(state):
        """Ignore caster support contact while retaining wall/obstacle hits."""
        names = (state.collision1_name.lower(), state.collision2_name.lower())
        return any(
            term in name
            for name in names
            for term in ("floor", "ground_plane", "ground::")
        )

    def _check_release(self):
        if not self._in_contact:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_contact_ns) / 1e9 < self._release_seconds:
            return
        duration = max(
            0.0, (self._last_contact_ns - self._contact_started_ns) / 1e9
        )
        self._in_contact = False
        self._publish_contact(False)
        if self._last_contact_state is not None:
            self._record_contact(
                self._last_contact_state,
                self._last_contact_ns,
                "contact_end",
                create_flag=False,
            )
        self.get_logger().info(
            f"PHYSICAL_CONTACT_CLEARED episode="
            f"{self._contact_episode_count} duration={duration:.3f}s"
        )

    def _publish_contact(self, active):
        message = Bool()
        message.data = active
        self._contact_pub.publish(message)

    @staticmethod
    def _finite_min(values):
        finite = [
            value for value in values
            if math.isfinite(value) and value > 0.0
        ]
        return min(finite) if finite else math.nan

    def _scan_sector(self, center, half_width, scan=None):
        scan = scan or self._scan
        if scan is None or not scan.ranges:
            return math.nan
        values = []
        angle = scan.angle_min
        for distance in scan.ranges:
            difference = math.atan2(
                math.sin(angle - center), math.cos(angle - center)
            )
            if abs(difference) <= half_width:
                values.append(distance)
            angle += scan.angle_increment
        return self._finite_min(values)

    def _pre_contact_scan(self, now_ns):
        if not self._scan_history:
            return (math.nan,) * 6
        target_ns = now_ns - int(self._lookback_seconds * 1e9)
        sample = min(
            self._scan_history, key=lambda item: abs(item[0] - target_ns)
        )
        latest_age = max(0.0, (now_ns - self._scan_ns) / 1e9)
        sampled_lookback = max(0.0, (now_ns - sample[0]) / 1e9)
        return (*sample[1:], latest_age, sampled_lookback)

    @staticmethod
    def _vector_xyz(vectors):
        if not vectors:
            return (math.nan, math.nan, math.nan)
        vector = vectors[0]
        return (vector.x, vector.y, vector.z)

    def _active_run(self):
        runs_dir = self._log_dir.parent / "runs"
        if not runs_dir.is_dir():
            return ""
        candidates = sorted(
            runs_dir.glob("*/metadata.json"),
            key=lambda path: path.stat().st_mtime,
        )
        for metadata_path in reversed(candidates):
            try:
                metadata = json.loads(
                    metadata_path.read_text(encoding="utf-8")
                )
            except (OSError, ValueError):
                continue
            if metadata.get("status") == "recording":
                return str(metadata_path.parent)
        return ""

    def _append_row(self, path, header, row):
        new_file = not path.exists() or path.stat().st_size == 0
        with path.open("a", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            if new_file:
                writer.writerow(header)
            writer.writerow(row)

    def _ensure_log_files(self):
        path = self._log_dir / "contact_events.csv"
        if not path.exists() or path.stat().st_size == 0:
            with path.open("w", newline="", encoding="utf-8") as stream:
                csv.writer(stream).writerow(CONTACT_HEADER)
        self._write_summary()

    def _read_summary(self):
        path = self._log_dir / "contact_summary.json"
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, ValueError, TypeError):
            return {}

    def _write_summary(self):
        summary = {
            "logging_active": True,
            "contact_episodes": self._contact_episode_count,
            "event_records": self._event_record_count,
            "currently_in_contact": self._in_contact,
            "updated_utc": datetime.now(timezone.utc).isoformat(),
        }
        (self._log_dir / "contact_summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8"
        )

    def _record_contact(self, state, now_ns, event, create_flag):
        self._last_flag_ns = now_ns
        self._event_record_count += 1
        robot_collision = state.collision1_name
        environment_collision = state.collision2_name
        robot_is_second = (
            not self._is_robot_collision(robot_collision)
            and self._is_robot_collision(environment_collision)
        )
        if robot_is_second:
            robot_collision, environment_collision = (
                environment_collision, robot_collision
            )
        position = self._vector_xyz(state.contact_positions)
        normal = self._vector_xyz(state.contact_normals)
        total_force = math.sqrt(
            state.total_wrench.force.x ** 2
            + state.total_wrench.force.y ** 2
            + state.total_wrench.force.z ** 2
        )
        odom_position = self._odom.pose.pose.position if self._odom else None
        odom_twist = self._odom.twist.twist if self._odom else None
        command_linear = (
            self._command.linear.x if self._command else math.nan
        )
        command_angular = (
            self._command.angular.z if self._command else math.nan
        )
        front, front_right, right, minimum, scan_age, lookback = \
            self._pre_contact_scan(now_ns)
        duration = max(0.0, (now_ns - self._contact_started_ns) / 1e9)
        utc = datetime.now(timezone.utc).isoformat()
        identifier = (
            f"AUTO_CONTACT_{self._contact_episode_count:04d} "
            f"{self._behavior[:32]} "
            f"clearance_{minimum:.3f}m"
        )[:80]
        active_run = self._active_run()

        if create_flag:
            self._append_row(
                self._log_dir / "user_log_flags.csv", FLAG_HEADER,
                [utc, now_ns, identifier, active_run, "automatic contact monitor"],
            )
        self._append_row(
            self._log_dir / "contact_events.csv", CONTACT_HEADER,
            [
                utc, now_ns, event, f"{duration:.3f}",
                self._contact_episode_count, self._event_record_count,
                self._behavior,
                robot_collision, environment_collision, *position, *normal,
                f"{total_force:.3f}",
                odom_position.x if odom_position else math.nan,
                odom_position.y if odom_position else math.nan,
                odom_twist.linear.x if odom_twist else math.nan,
                odom_twist.angular.z if odom_twist else math.nan,
                command_linear, command_angular,
                front, front_right, right, minimum, scan_age, lookback,
            ],
        )
        if active_run and create_flag:
            self._append_row(
                Path(active_run) / "events.csv",
                [
                    "utc_timestamp", "ros_time_nanoseconds", "event",
                    "operator", "scenario",
                ],
                [
                    utc, now_ns, f"physical_contact:{identifier}",
                    "automatic", self._behavior,
                ],
            )
        self._write_summary()
        if create_flag:
            event_message = String()
            event_message.data = identifier
            self._event_pub.publish(event_message)
            self.get_logger().error(
                "PHYSICAL_CONTACT_FLAG "
                f"episode={self._contact_episode_count} id='{identifier}' "
                f"behavior='{self._behavior}' "
                f"environment='{environment_collision}' force={total_force:.2f}N "
                f"lidar_front={front:.3f}m lidar_right={right:.3f}m "
                f"lidar_min={minimum:.3f}m "
                f"cmd=({command_linear:.3f},{command_angular:.3f})"
            )

    @staticmethod
    def _is_robot_collision(name):
        return any(
            part in name
            for part in (
                "base_link_collision", "front_caster_collision",
                "left_rear_wheel_collision", "right_rear_wheel_collision",
            )
        )


def main(args=None):
    """Run the contact monitor until ROS shuts down."""
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
