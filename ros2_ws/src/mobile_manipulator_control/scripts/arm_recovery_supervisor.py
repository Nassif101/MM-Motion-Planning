#!/usr/bin/env python3
"""Re-activate the Unity arm hardware and its controllers after a stale-feedback pause.

The UnityArm hardware deactivates when Unity's arm feedback pauses for more than its
0.5 s state timeout (ADR 0005); the controller manager then also deactivates the arm
controllers, aborting any trajectory. Unity's own watchdog holds the arm meanwhile. Once
fresh feedback with advancing stamps has flowed for `fresh_for_s`, this node activates the
hardware again (which restarts commands from the actual joint positions) and then the
controllers. A new simulation epoch stays latched in the hardware: activation is refused
and arm control must be restarted. Runs on wall time so a /clock pause cannot stop it.
"""
import time

import rclpy
from controller_manager_msgs.srv import (ListHardwareComponents, SetHardwareComponentState,
                                         SwitchController)
from lifecycle_msgs.msg import State
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class ArmRecoverySupervisor(Node):
    def __init__(self):
        super().__init__("arm_recovery_supervisor")
        self.hardware = self.declare_parameter("hardware", "UnityArm").value
        self.controllers = list(self.declare_parameter(
            "controllers", ["arm_joint_state_broadcaster", "arm_controller"]).value)
        self.fresh_for_s = self.declare_parameter("fresh_for_s", 1.0).value
        self.retry_s = self.declare_parameter("retry_s", 3.0).value
        self.last_state, self.last_stamp, self.fresh_since = None, -1.0, None
        self.last_attempt, self.busy, self.recoveries = 0.0, False, 0
        self.create_subscription(JointState, "/arm/state", self.on_state,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        prefix = "/controller_manager/"
        self.list_client = self.create_client(ListHardwareComponents, prefix + "list_hardware_components")
        self.set_client = self.create_client(SetHardwareComponentState, prefix + "set_hardware_component_state")
        self.switch_client = self.create_client(SwitchController, prefix + "switch_controller")
        self.create_timer(0.5, self.tick)

    def on_state(self, msg):
        now = time.monotonic()
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= self.last_stamp or (self.last_state is not None and now - self.last_state > 0.2):
            self.fresh_since = None  # regression, repeat, or a gap: restart the freshness window
        if stamp > self.last_stamp:
            if self.fresh_since is None:
                self.fresh_since = now
            self.last_stamp = stamp
        self.last_state = now

    def fresh(self):
        now = time.monotonic()
        return (self.last_state is not None and now - self.last_state < 0.2 and
                self.fresh_since is not None and now - self.fresh_since >= self.fresh_for_s)

    def tick(self):
        if self.busy or not self.list_client.service_is_ready():
            return
        self.busy = True
        self.list_client.call_async(ListHardwareComponents.Request()).add_done_callback(self.on_list)

    def on_list(self, future):
        result = future.result()
        component = next((c for c in (result.component if result else []) if c.name == self.hardware), None)
        if (component is None or component.state.id == State.PRIMARY_STATE_ACTIVE or not self.fresh()
                or time.monotonic() - self.last_attempt < self.retry_s):
            self.busy = False
            return
        self.last_attempt = time.monotonic()
        self.get_logger().warn(f"{self.hardware} is {component.state.label} with fresh Unity feedback; re-activating")
        request = SetHardwareComponentState.Request(
            name=self.hardware, target_state=State(id=State.PRIMARY_STATE_ACTIVE, label="active"))
        self.set_client.call_async(request).add_done_callback(self.on_set)

    def on_set(self, future):
        result = future.result()
        if result is None or not result.ok:
            self.get_logger().error(f"{self.hardware} refused activation; if the simulation epoch changed, "
                                    "restart arm control")
            self.busy = False
            return
        request = SwitchController.Request(activate_controllers=self.controllers,
                                           strictness=SwitchController.Request.STRICT, activate_asap=True)
        request.timeout.sec = 5
        self.switch_client.call_async(request).add_done_callback(self.on_switch)

    def on_switch(self, future):
        result = future.result()
        if result is not None and result.ok:
            self.recoveries += 1
            self.get_logger().warn(f"Arm control recovered ({self.recoveries} since start)")
        else:
            self.get_logger().error(f"Could not re-activate {self.controllers}: "
                                    f"{result.message if result else 'no response'}")
        self.busy = False


def main():
    rclpy.init()
    node = ArmRecoverySupervisor()
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
