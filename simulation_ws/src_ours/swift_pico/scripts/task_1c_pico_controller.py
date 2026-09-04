#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_controller which holds the position of Swift Pico Drone.
This node publishes and subscribes to the following topics:
    PUBLICATIONS            SUBSCRIPTIONS
    /drone_command          /whycon/poses
    /pid_error              
'''

# Importing the required libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from error_msg.msg import Error
import rclpy
from rclpy.node import Node

class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # Initialize ROS node

        # Current position of drone [x, y, z], updated in whycon_callback
        self.current_state = [0.0, 0.0, 0.0]
        # ------------------------------------------------------------------
        # 1. EMA filter buffers (smoothed pose).  Alpha = 0.4 works well for
        #    30 Hz WhyCon data – adjust only if you need a different cutoff.
        # ------------------------------------------------------------------
        self.smoothed_state = [0.0, 0.0, 0.0]
        self.filter_alpha = 0.4   # 0.0 = no filtering, 1.0 = raw input

        # Setpoint to reach and hold [x, y, z]
        self.desired_state = [-7, 0, 20]

        # Command message (SwiftMsgs) with initial neutral values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # ==================================================================
        # HARDCODED PID VALUES - SET YOUR DESIRED VALUES HERE
        # ==================================================================
        # PID gains for [roll, pitch, throttle]
        # Format: [Kp_roll, Kp_pitch, Kp_throttle], etc.
        self.Kp = [-10,-41, -70.95]    # Example values - adjust as needed
        self.Ki = [-1, -1, -4] # Example values - adjust as needed  
        self.Kd = [-80, -40,-53 ]    # Example values - adjust as needed
        # ==================================================================

        # PID variables
        self.prev_error = [0.0, 0.0, 0.0]  # Previous errors [roll, pitch, throttle]
        self.error_sum = [0.0, 0.0, 0.0]   # Integral sums [roll, pitch, throttle]
        self.max_values = [2000, 2000, 2000]  # Max RC values
        self.min_values = [1000, 1000, 1000]  # Min RC values
        self.pos_error = Error()  # Error message for position errors

        # Sample time for PID (30Hz)
        self.sample_time = 0.033

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/pos_error', 10)

        # Subscribers - ONLY whycon poses, no PID subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        
        # Timer for PID loop
        self.timer = self.create_timer(self.sample_time, self.pid)

        self.arm()  # Arm the drone
        self.get_logger().info("Drone controller started with hardcoded PID values")
        self.get_logger().info(f"Roll PID: Kp={self.Kp[0]}, Ki={self.Ki[0]}, Kd={self.Kd[0]}")
        self.get_logger().info(f"Pitch PID: Kp={self.Kp[1]}, Ki={self.Ki[1]}, Kd={self.Kd[1]}")
        self.get_logger().info(f"Throttle PID: Kp={self.Kp[2]}, Ki={self.Ki[2]}, Kd={self.Kd[2]}")

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)
        self.get_logger().info("Drone armed with rc_aux4=2000")

    def whycon_callback(self, msg):
        # ------------------------------------------------------------------
        # 1. Update raw pose
        # ------------------------------------------------------------------
        raw = [msg.poses[0].position.x,
               msg.poses[0].position.y,
               msg.poses[0].position.z]
        self.current_state[0] = raw[0]
        self.current_state[1] = raw[1]
        self.current_state[2] = raw[2]

        # ------------------------------------------------------------------
        # 2. Apply EMA filter (noise reduction)
        # ------------------------------------------------------------------
        for i in range(3):
            self.smoothed_state[i] = (self.filter_alpha * raw[i] +
                                     (1.0 - self.filter_alpha) * self.smoothed_state[i])

    def pid(self):
        # Compute errors [roll, pitch, throttle] based on desired vs. current state
        error = [0.0, 0.0, 0.0]
        # ------------------------------------------------------------------
        # Use *filtered* pose for control – this removes jitter that keeps
        # the drone oscillating forever.
        # ------------------------------------------------------------------
        error[0] = self.desired_state[1] - self.smoothed_state[1]  # Roll error (y-error)
        error[1] = self.desired_state[0] - self.smoothed_state[0]  # Pitch error (x-error)
        error[2] = self.desired_state[2] - self.smoothed_state[2]  # Throttle error (z-error)

        # Compute PID outputs [roll, pitch, throttle]
        output = [0.0, 0.0, 0.0]
        for i in range(3):
            # Derivative term
            derivative = (error[i] - self.prev_error[i]) / self.sample_time
            # Integral term with anti-windup (clamp to avoid overflow)
            self.error_sum[i] += error[i] * self.sample_time
            if self.error_sum[i] > 10.0:  # Arbitrary limit to prevent windup
                self.error_sum[i] = 10.0
            elif self.error_sum[i] < -10.0:
                self.error_sum[i] = -10.0
            # PID output
            output[i] = (self.Kp[i] * error[i] +
                         self.Ki[i] * self.error_sum[i] +
                         self.Kd[i] * derivative)
            # Update previous error
            self.prev_error[i] = error[i]

        # Map outputs to commands (cast to int for SwiftMsgs)
        self.cmd.rc_roll = int(1500 - output[0])      # Roll for y-control (inverted for correct direction)
        self.cmd.rc_pitch = int(1500 - output[1])     # Pitch for x-control (inverted for correct direction)
        self.cmd.rc_throttle = int(1500 + output[2])  # Throttle for z-control

        # ------------------------------------------------------------------
        # 2. Take-off boost for tiny Ki values
        # ------------------------------------------------------------------
        if self.Ki[2] < 1e-4 and abs(error[2]) > 3.5:
            boost_throttle = 1633
            self.get_logger().info(f"Applying take-off boost (Ki={self.Ki[2]:.2e}), throttle forced to {boost_throttle}")
            self.cmd.rc_throttle = boost_throttle

        # Clamp commands to min/max values
        self.cmd.rc_roll = max(self.min_values[0], min(self.max_values[0], self.cmd.rc_roll))
        self.cmd.rc_pitch = max(self.min_values[1], min(self.max_values[1], self.cmd.rc_pitch))
        self.cmd.rc_throttle = max(self.min_values[2], min(self.max_values[2], self.cmd.rc_throttle))

        # Publish position errors (corrected order: x, y, z)
        self.pos_error.roll_error = error[0]  # Roll error (y-error)
        self.pos_error.pitch_error = error[1] # Pitch error (x-error)
        self.pos_error.throttle_error = error[2] # Throttle error (z-error)
        self.pos_error.yaw_error = 0.0
        self.pos_error_pub.publish(self.pos_error)

        # Publish commands
        self.command_pub.publish(self.cmd)

def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    try:
        rclpy.spin(swift_pico)
    except KeyboardInterrupt:
        swift_pico.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        swift_pico.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
