#!/usr/bin/env python3

# This python file runs a ROS 2-node of name waypoint_server which implements an action server to navigate the Swift Pico Drone to the given waypoints.

import time
import math
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Import control specific libraries
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray, PoseStamped
from error_msg.msg import Error
from controller_msg.msg import PIDTune
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

# Import the action
from waypoint_navigation.action import NavToWaypoint
from geometry_msgs.msg import Point

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        self.pid_or_lqr_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()
        self.odometry_callback_group = ReentrantCallbackGroup()

        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.duration = 0

        self.yaw = 0.0
        self.xyz = [0.0, 0.0, 0.0, 0.0]
        self.dtime = 0

        # PID Controller Variables (UPDATED with your new PID values)
        self.current_state = [0.0, 0.0, 0.0]
        self.smoothed_state = [0.0, 0.0, 0.0]
        self.filter_alpha = 0.13   # UPDATED: from 0.22 to 0.2
        self.desired_state = [0.0, 0.0, 0.0]  # Will be set by action goals
        
        # UPDATED PID gains from your new controller
        self.Kp = [-41, -41, -76]     # Same Kp
        self.Ki = [-2.5, -2.5, -4]    # Same Ki  
        self.Kd = [-80, -60, -88]     # UPDATED: Better derivative gains
        
        self.prev_error = [0.0, 0.0, 0.0]
        self.error_sum = [0.0, 0.0, 0.0]
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]
        self.pos_error = Error()  # ADDED: Error message for publishing

        # Declaring a cmd of message type swift_msgs and initializing values
        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500  # UPDATED: from 1400 to 1500 for better starting point

        self.sample_time = 0.033  # 30Hz from your PID

        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pos_error_pub = self.create_publisher(Error, '/position_error', 10)  # UPDATED: topic name

        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10, callback_group=self.odometry_callback_group)

        # Create action server for 'waypoint_navigation'
        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback,
            callback_group=self.action_callback_group
        )
        
        self.arm()
        
        # Define the function to be run inside the timer callback
        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_or_lqr_callback_group)
        
        self.get_logger().info("Waypoint Server started with UPDATED PID controller")
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

    # Whycon callback function
    def whycon_callback(self, msg):
        if not msg.poses or len(msg.poses) == 0:
            self.get_logger().warn("No poses received")
            return
        else:
            # Extract current position from whycon
            raw = [msg.poses[0].position.x,
                   msg.poses[0].position.y, 
                   msg.poses[0].position.z]
            
            self.current_state[0] = raw[0]
            self.current_state[1] = raw[1]
            self.current_state[2] = raw[2]

            # Apply EMA filter with UPDATED alpha
            for i in range(3):
                self.smoothed_state[i] = (self.filter_alpha * raw[i] +
                                         (1.0 - self.filter_alpha) * self.smoothed_state[i])
            
            self.dtime = time.time()

    def odometry_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.roll_deg = math.degrees(roll)
        self.pitch_deg = math.degrees(pitch)
        self.yaw_deg = math.degrees(yaw)
        self.yaw = self.yaw_deg

    # UPDATED PID Controller function with your improvements
    def pid(self):
        # Compute errors [roll, pitch, throttle] based on desired vs. current state
        error = [0.0, 0.0, 0.0]
        error[0] = self.desired_state[1] - self.smoothed_state[1]  # Roll error (y-error)
        error[1] = self.desired_state[0] - self.smoothed_state[0]  # Pitch error (x-error)
        error[2] = self.desired_state[2] - self.smoothed_state[2]  # Throttle error (z-error)

        # Compute PID outputs [roll, pitch, throttle]
        output = [0.0, 0.0, 0.0]
        for i in range(3):
            # Derivative term
            derivative = (error[i] - self.prev_error[i]) / self.sample_time
            # Integral term with anti-windup (UPDATED with your limits)
            self.error_sum[i] += error[i] * self.sample_time
            if self.error_sum[i] > 10.0:  # Arbitrary limit to prevent windup
                self.error_sum[i] = 10.0
            elif self.error_sum[i] < -10.0:
                self.error_sum[i] = -10.0
            # PID output with UPDATED Kd values
            output[i] = (self.Kp[i] * error[i] +
                         self.Ki[i] * self.error_sum[i] +
                         self.Kd[i] * derivative)
            # Update previous error
            self.prev_error[i] = error[i]

        # Map outputs to commands
        self.cmd.rc_roll = int(1500 - output[0])
        self.cmd.rc_pitch = int(1500 - output[1])
        self.cmd.rc_throttle = int(1500 + output[2])

        # Take-off boost for tiny Ki values (IMPROVED with logging)
        if self.Ki[2] < 1e-4 and abs(error[2]) > 3.5:
            boost_throttle = 1633
            self.get_logger().info(f"Applying take-off boost (Ki={self.Ki[2]:.2e}), throttle forced to {boost_throttle}")
            self.cmd.rc_throttle = boost_throttle

        # Clamp commands to min/max values
        self.cmd.rc_roll = max(self.min_values[0], min(self.max_values[0], self.cmd.rc_roll))
        self.cmd.rc_pitch = max(self.min_values[1], min(self.max_values[1], self.cmd.rc_pitch))
        self.cmd.rc_throttle = max(self.min_values[2], min(self.max_values[2], self.cmd.rc_throttle))

        # Publish position errors (UPDATED with proper error message)
        self.pos_error.roll_error = error[0]  # Roll error (y-error)
        self.pos_error.pitch_error = error[1] # Pitch error (x-error)
        self.pos_error.throttle_error = error[2] # Throttle error (z-error)
        self.pos_error.yaw_error = 0.0
        self.pos_error_pub.publish(self.pos_error)

        # Publish commands
        self.command_pub.publish(self.cmd)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Set desired state from goal
        self.desired_state[0] = goal_handle.request.waypoint.x
        self.desired_state[1] = goal_handle.request.waypoint.y
        self.desired_state[2] = goal_handle.request.waypoint.z

        self.get_logger().info(f'New Waypoint Set: {self.desired_state}')
        
        # Reset stabilization variables
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.time_inside_sphere = 0
        self.duration = self.dtime

        # Create feedback object
        feedback_msg = NavToWaypoint.Feedback()
        
        # Check stabilization at waypoint
        while True:
            # Current whycon poses for feedback
            feedback_msg.current_waypoint = PoseStamped()
            feedback_msg.current_waypoint.header = Header()
            feedback_msg.current_waypoint.header.stamp = self.get_clock().now().to_msg()
            feedback_msg.current_waypoint.pose.position.x = self.smoothed_state[0]
            feedback_msg.current_waypoint.pose.position.y = self.smoothed_state[1]
            feedback_msg.current_waypoint.pose.position.z = self.smoothed_state[2]
            feedback_msg.current_waypoint.header.stamp.sec = int(self.max_time_inside_sphere)

            goal_handle.publish_feedback(feedback_msg)

            drone_is_in_sphere = self.is_drone_in_sphere(self.smoothed_state, goal_handle, 0.5)  # UPDATED: Exact 0.4 requirement

            if not drone_is_in_sphere and self.point_in_sphere_start_time is None:
                pass
            elif drone_is_in_sphere and self.point_in_sphere_start_time is None:
                self.point_in_sphere_start_time = self.dtime
                self.get_logger().info('Drone in sphere for 1st time')
            elif drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time
                if self.time_inside_sphere > self.max_time_inside_sphere:
                    self.max_time_inside_sphere = self.time_inside_sphere
            elif not drone_is_in_sphere and self.point_in_sphere_start_time is not None:
                self.get_logger().info('Drone out of sphere')
                self.point_in_sphere_start_time = None

            if self.max_time_inside_sphere >= 3:  # 3 seconds stabilization
                break
                
            time.sleep(0.1)  # Small delay to prevent busy waiting

        goal_handle.succeed()

        # Create result object
        result = NavToWaypoint.Result()
        result.hov_time = self.dtime - self.duration
        self.get_logger().info(f'Waypoint completed in {result.hov_time:.2f} seconds')
        
        return result

    def is_drone_in_sphere(self, drone_pos, goal_handle, radius):
        return (
            (drone_pos[0] - goal_handle.request.waypoint.x) ** 2
            + (drone_pos[1] - goal_handle.request.waypoint.y) ** 2
            + (drone_pos[2] - goal_handle.request.waypoint.z) ** 2
        ) <= radius**2

def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
