#!/usr/bin/env python3
"""Single command arbiter for wall, frontier, and recovery behaviors."""

from enum import Enum

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import rclpy
from rclpy.node import Node


class State(Enum):
    WALL_FOLLOWING = "WALL_FOLLOWING"
    FRONTIER_NAVIGATION = "FRONTIER_NAVIGATION"
    RECOVERY = "RECOVERY"


class ExplorationCoordinator(Node):
    """Evaluate transitions and permit exactly one velocity source."""

    def __init__(self):
        super().__init__("exploration_coordinator")
        self.declare_parameter("command_timeout", 0.35)
        self.declare_parameter("recovery_settle_time", 2.0)
        # Nav2 remains usable before the optional wall behavior starts.
        self._state = State.FRONTIER_NAVIGATION
        self._resume_state = self._state
        self._handoff_active = False
        self._wall_state = "inactive"
        self._commands = {"wall": Twist(), "nav": Twist(), "recovery": Twist()}
        self._command_times = {key: None for key in self._commands}
        self._recovery_started = None
        self._output = self.create_publisher(Twist, "/cmd_vel", 10)
        self._state_output = self.create_publisher(
            String, "/exploration_coordination_state", 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_wall", lambda msg: self._command("wall", msg), 10
        )
        self.create_subscription(
            Twist, "/cmd_vel_navigation",
            lambda msg: self._command("nav", msg), 10,
        )
        self.create_subscription(
            Twist, "/cmd_vel_behaviors",
            lambda msg: self._command("recovery", msg), 10,
        )
        self.create_subscription(
            Bool, "/frontier_handoff_requested", self._handoff, 10
        )
        self.create_subscription(
            Bool, "/wall_tracing_active", self._wall_active, 10
        )
        self.create_subscription(
            Bool, "/frontier_navigation_complete", self._frontier_complete, 10
        )
        self.create_subscription(Bool, "/physical_contact", self._contact, 10)
        self.create_subscription(
            String, "/wall_behavior_state", self._wall_state_changed, 10
        )
        self.create_timer(0.05, self._evaluate)
        self.get_logger().info(
            "Coordinator owns /cmd_vel; Nav2 is enabled until wall control claims it."
        )

    def _command(self, source, message):
        self._commands[source] = message
        self._command_times[source] = self.get_clock().now()
        if source == "recovery" and self._moving(message):
            if self._state != State.RECOVERY:
                self._resume_state = self._state
                self._transition(State.RECOVERY, "Nav2 recovery command")
            self._recovery_started = self.get_clock().now()

    def _handoff(self, message):
        self._handoff_active = message.data
        if message.data and self._state == State.WALL_FOLLOWING:
            self._transition(
                State.FRONTIER_NAVIGATION,
                "wall behavior confirmed lack of mapping progress",
            )

    def _wall_active(self, message):
        if (
            message.data
            and not self._handoff_active
            and self._state == State.FRONTIER_NAVIGATION
        ):
            self._transition(State.WALL_FOLLOWING, "wall behavior claimed control")

    def _frontier_complete(self, message):
        if message.data and self._state == State.FRONTIER_NAVIGATION:
            self._handoff_active = False
            self._transition(State.WALL_FOLLOWING, "frontier goal completed")

    def _contact(self, message):
        if message.data and self._state != State.RECOVERY:
            self._resume_state = self._state
            self._recovery_started = self.get_clock().now()
            self._transition(State.RECOVERY, "physical-contact safety event")

    def _wall_state_changed(self, message):
        self._wall_state = message.data

    def _transition(self, state, reason):
        if state == self._state:
            return
        previous = self._state
        self._output.publish(Twist())
        self._state = state
        self.get_logger().info(f"{previous.value} -> {state.value}: {reason}")

    @staticmethod
    def _moving(command):
        return abs(command.linear.x) > 1e-4 or abs(command.angular.z) > 1e-4

    def _fresh(self, source, now):
        stamp = self._command_times[source]
        return stamp is not None and (
            now - stamp
        ).nanoseconds / 1e9 <= float(
            self.get_parameter("command_timeout").value
        )

    def _evaluate(self):
        now = self.get_clock().now()
        if self._state == State.RECOVERY:
            if self._fresh("recovery", now):
                command = self._commands["recovery"]
            else:
                command = Twist()
            if self._recovery_started is not None and not self._moving(command):
                quiet = (now - self._recovery_started).nanoseconds / 1e9
                if quiet >= float(
                    self.get_parameter("recovery_settle_time").value
                ):
                    self._transition(self._resume_state, "recovery settled")
        elif self._state == State.FRONTIER_NAVIGATION:
            command = self._commands["nav"] if self._fresh("nav", now) else Twist()
        else:
            # Wall acquisition/alignment deliberately uses Nav2. Direct wall
            # states use the isolated wall channel.
            nav_alignment = self._wall_state in {
                "claiming_nav2_for_wall_alignment",
                "nav2_align_parallel_to_right_wall",
            }
            source = "nav" if nav_alignment else "wall"
            command = self._commands[source] if self._fresh(source, now) else Twist()
        self._output.publish(command)
        self._state_output.publish(String(data=self._state.value))


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._output.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
