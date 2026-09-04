#!/usr/bin/env python3
'''
ROS2 PID Controller for Swift Pico - Task 4B (real hardware)
- Uses /drone/cmd/arming service + aux1 fallback
- Forces low throttle during arming sequence
- Safety clamp on throttle after arming
- Added simple EMA filter on position
- Conditional Ki boost for throttle when positive error > 2m
'''

import rclpy
from rclpy.node import Node
import time

from rc_msgs.msg import RCMessage
from geometry_msgs.msg import PoseArray
from controller_msg.msg import PIDTune
from error_msg.msg import Error
from rc_msgs.srv import CommandBool


class SwiftPicoPIDController(Node):
    def __init__(self):
        super().__init__('pico_controller')

        # ─── States ─────────────────────────────────────────────────────────────
        self.current_state = [0.0, 0.0, 0.0]   # raw x, y, z from WhyCon
        self.smoothed_state = [0.0, 0.0, 0.0]  # filtered version
        self.filter_alpha = 0.2                # EMA smoothing factor

        self.desired_state = [0.0, -5.5, 19.0]  # target [x, y, z]

        # ─── RC command ─────────────────────────────────────────────────────────
        self.cmd = RCMessage()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1000

        # ─── PID Gains ──────────────────────────────────────────────────────────
        self.Kp = [0.0, 0.0, 0.0]
        self.Ki = [0.0, 0.0, 0.0]
        self.Kd = [0.0, 0.0, 0.0]

        # Throttle special handling - conditional Ki boost
        self.normal_ki_throttle = 0.008     # base Ki multiplier for throttle
        self.boost_ki_multiplier = -3     # how much stronger when needed
        self.boost_error_threshold = 2    # meters - when to apply boost

        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.sample_time = 0.033

        self.is_armed = False

        # ─── Safety throttle limits ─────────────────────────────────────────────
        self.throttle_min_safe = 1300
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        # ─── Publishers & Subscribers ───────────────────────────────────────────
        self.command_pub = self.create_publisher(RCMessage, '/rc_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)

        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 10)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 10)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 10)
        self.create_subscription(PIDTune, "/throttle_pid", self.throttle_set_pid, 10)

        # Arming service
        self.arm_client = self.create_client(CommandBool, '/drone/cmd/arming')
        self.wait_for_arm_service()

        # PID timer
        self.timer = self.create_timer(self.sample_time, self.pid)

        self.get_logger().info("PID Controller Initialized (EMA + conditional throttle Ki boost)")
        self.arm_drone()  # ← uncomment ONLY when you're ready!

    def wait_for_arm_service(self):
        timeout = 12.0
        start = time.time()
        while rclpy.ok() and not self.arm_client.wait_for_service(timeout_sec=1.0):
            if time.time() - start > timeout:
                self.get_logger().error("Arming service not available!")
                return
            self.get_logger().info("Waiting for arming service...")

    def arm_drone(self):
        self.get_logger().info("Starting arming sequence...")

        # Step 1: Send low throttle + disarm state
        for _ in range(8):
            self.cmd.rc_throttle = 1000
            self.cmd.aux1 = 1000
            self.command_pub.publish(self.cmd)
            time.sleep(0.12)

        # Step 2: Call arming service
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)

        if future.result() is not None:
            self.get_logger().info("Arming service call completed")
        else:
            self.get_logger().warn("Arming service timeout")

        # Step 3: Force aux1 = 2000
        self.get_logger().info("Sending aux1=2000 for 2 seconds...")
        for _ in range(20):
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = 1000
            self.cmd.aux1 = 2000
            self.command_pub.publish(self.cmd)
            time.sleep(0.1)

        self.is_armed = True
        self.get_logger().info("Arming sequence finished - check for beeps/motors ready")

    def disarm_drone(self):
        req = CommandBool.Request()
        req.value = False
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        self.is_armed = False
        self.get_logger().info("Disarm requested via service")

        # Final safety packet
        self.cmd.rc_throttle = 1000
        self.cmd.aux1 = 1000
        self.command_pub.publish(self.cmd)

    # ─── Callbacks ──────────────────────────────────────────────────────────────
    def whycon_callback(self, msg):
        if msg.poses:
            p = msg.poses[0].position
            self.current_state = [p.x, p.y, p.z]

            # EMA filter
            for i in range(3):
                self.smoothed_state[i] = (
                    self.filter_alpha * self.current_state[i] +
                    (1.0 - self.filter_alpha) * self.smoothed_state[i]
                )

    def roll_set_pid(self, tune):
        self.Kp[0] = -200 * 0.006
        self.Ki[0] = -1 * 0.008
        self.Kd[0] = -400 * 0.03

    def pitch_set_pid(self, tune):
        self.Kp[1] =  -200* 0.006
        self.Ki[1] = -1 * 0.008
        self.Kd[1] = -300 * 0.03

    def throttle_set_pid(self, tune):
        self.Kp[2] = -300 * 0.03
        self.Ki[2] = -1 * self.normal_ki_throttle   # base value used here
        self.Kd[2] = -70 * 0.06

    # ─── Main PID Loop ──────────────────────────────────────────────────────────
    def pid(self):
        error = [
            self.desired_state[0] - self.smoothed_state[0],
            self.desired_state[1] - self.smoothed_state[1],
            self.desired_state[2] - self.smoothed_state[2]
        ]

        output = [0.0, 0.0, 0.0]

        for i in range(3):
            d_error = error[i] - self.prev_error[i]

            # Decide which Ki to use (only special for throttle/z)
            ki_to_use = self.Ki[i]

            if i == 3:  # throttle channel
                if error[2] > self.boost_error_threshold:
                    ki_to_use = self.Ki[2] * self.boost_ki_multiplier
                    # Optional: you can log when boost activates
                    # self.get_logger().info(f"Throttle Ki boost active! error={error[2]:.2f}")

            # Integral update (you can add anti-windup here later)
            self.error_sum[i] += error[i] * self.sample_time

            output[i] = (
                self.Kp[i] * error[i] +
                ki_to_use * self.error_sum[i] +
                self.Kd[i] * (d_error / self.sample_time)
            )

            self.prev_error[i] = error[i]

        # Apply control outputs
        if not self.is_armed:
            self.cmd.rc_roll = 1500
            self.cmd.rc_pitch = 1500
            self.cmd.rc_yaw = 1500
            self.cmd.rc_throttle = 1000
        else:
            self.cmd.rc_roll = int(1500 + output[0])
            self.cmd.rc_pitch = int(1400 + output[1])
            self.cmd.rc_throttle = int(1450 + output[2])   # 1490 is common hover bias
            self.cmd.rc_yaw = 1500

            # Safety: prevent auto-disarm
            if self.cmd.rc_throttle < self.throttle_min_safe:
                self.cmd.rc_throttle = self.throttle_min_safe

        # Final clamping - always do this!
        self.cmd.rc_roll = max(1000, min(2000, self.cmd.rc_roll))
        self.cmd.rc_pitch = max(1000, min(2000, self.cmd.rc_pitch))
        self.cmd.rc_throttle = max(1000, min(2000, self.cmd.rc_throttle))

        self.command_pub.publish(self.cmd)

        # Publish errors (using raw error for visualization)
        err = Error()
        err.roll_error = error[0]
        err.pitch_error = error[1]
        err.throttle_error = error[2]
        err.yaw_error = 0.0
        self.pos_error_pub.publish(err)

    def destroy_node(self):
        if self.is_armed:
            self.disarm_drone()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = SwiftPicoPIDController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Keyboard interrupt - shutting down")
    finally:
        controller.disarm_drone()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
