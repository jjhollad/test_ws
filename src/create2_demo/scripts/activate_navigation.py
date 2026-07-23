#!/usr/bin/env python3
"""Activate Nav2 servers serially to avoid Humble lifecycle startup races."""

import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


NAVIGATION_NODES = (
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
)


class NavigationActivator(Node):
    def __init__(self):
        super().__init__("navigation_activator")

    def state(self, name, timeout=10.0):
        # Humble can lose the first lifecycle response while several composed
        # Nav2 servers are still constructing. Retry short requests until the
        # overall deadline instead of spending the whole timeout on one future
        # and permanently abandoning an otherwise healthy stack.
        deadline = time.monotonic() + timeout
        last_error = "service unavailable"
        while time.monotonic() < deadline and rclpy.ok():
            remaining = deadline - time.monotonic()
            client = self.create_client(GetState, f"/{name}/get_state")
            try:
                if not client.wait_for_service(timeout_sec=min(2.0, remaining)):
                    last_error = "service unavailable"
                    continue
                future = client.call_async(GetState.Request())
                rclpy.spin_until_future_complete(
                    self, future, timeout_sec=min(2.0, remaining)
                )
                if future.done() and future.result() is not None:
                    state = future.result().current_state
                    return state.id, state.label
                last_error = "response timed out"
                if not future.done():
                    client.remove_pending_request(future)
            finally:
                self.destroy_client(client)
            time.sleep(0.1)
        raise RuntimeError(
            f"/{name}/get_state did not respond before {timeout:.0f} s "
            f"({last_error})"
        )

    def transition(self, name, transition_id, target_state, target_label):
        client = self.create_client(ChangeState, f"/{name}/change_state")
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"/{name}/change_state did not become available")
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)

        # Verify actual state instead of trusting only the transition response;
        # overloaded Humble simulation can deliver the response late.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            state_id, _ = self.state(name, timeout=2.0)
            if state_id == target_state:
                self.get_logger().info(f"{name} is {target_label}.")
                return
            time.sleep(0.1)
        _, label = self.state(name)
        raise RuntimeError(
            f"{name} failed transition {transition_id}; current state is {label}"
        )

    def run(self):
        for name in NAVIGATION_NODES:
            state_id, label = self.state(name, timeout=30.0)
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f"Configuring {name}.")
                self.transition(
                    name,
                    Transition.TRANSITION_CONFIGURE,
                    State.PRIMARY_STATE_INACTIVE,
                    "inactive",
                )
            elif state_id not in (
                State.PRIMARY_STATE_INACTIVE,
                State.PRIMARY_STATE_ACTIVE,
            ):
                raise RuntimeError(f"{name} has unexpected state {label}")

        for name in NAVIGATION_NODES:
            state_id, label = self.state(name)
            # A server may expose its lifecycle services before construction
            # has completely settled. Re-check and configure here as well so
            # a transient early state cannot leave a server behind.
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f"Configuring {name}.")
                self.transition(
                    name,
                    Transition.TRANSITION_CONFIGURE,
                    State.PRIMARY_STATE_INACTIVE,
                    "inactive",
                )
                state_id, label = self.state(name)
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f"Activating {name}.")
                self.transition(
                    name,
                    Transition.TRANSITION_ACTIVATE,
                    State.PRIMARY_STATE_ACTIVE,
                    "active",
                )
            elif state_id != State.PRIMARY_STATE_ACTIVE:
                raise RuntimeError(f"{name} has unexpected state {label}")


def main():
    rclpy.init()
    node = NavigationActivator()
    exit_code = 0
    try:
        for attempt in range(1, 6):
            try:
                node.run()
                node.get_logger().info(
                    "All Nav2 navigation servers are active."
                )
                break
            except Exception as error:
                if attempt == 5:
                    raise
                node.get_logger().warn(
                    f"Nav2 activation attempt {attempt}/5 failed: {error}; "
                    "retrying the idempotent lifecycle sequence."
                )
                time.sleep(2.0)
    except Exception as error:  # launch must receive a nonzero exit on failure
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
