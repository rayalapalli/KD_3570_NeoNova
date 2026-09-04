#!/usr/bin/env python3

import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Import the action and service
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints
from geometry_msgs.msg import Point
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints

class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.goals = []
        self.goal_index = 0
        
        # Create action client for 'waypoint_navigation'
        self.action_client = ActionClient(self, NavToWaypoint, 'waypoint_navigation')
        
        # Create service client for 'waypoints'
        self.cli = self.create_client(GetWaypoints, 'waypoints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.get_logger().info("Waypoint Client initialized")

    ### Action client functions

    def send_goal(self, waypoint):
        # Create a NavToWaypoint goal object
        goal_msg = NavToWaypoint.Goal()
        goal_msg.waypoint = Point()
        goal_msg.waypoint.x = waypoint[0]
        goal_msg.waypoint.y = waypoint[1]
        goal_msg.waypoint.z = waypoint[2]

        # Wait for the action server to be available
        self.action_client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: [{waypoint[0]}, {waypoint[1]}, {waypoint[2]}]')
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )    
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: Stabilization time: {result.hov_time} seconds')

        self.goal_index += 1

        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')      

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        x = feedback.current_waypoint.pose.position.x
        y = feedback.current_waypoint.pose.position.y
        z = feedback.current_waypoint.pose.position.z
        t = feedback.current_waypoint.header.stamp.sec
        self.get_logger().info(f'Received feedback! Current position: [{x:.2f}, {y:.2f}, {z:.2f}]')
        self.get_logger().info(f'Max time inside sphere: {t} seconds')

    # Service client functions

    def send_request(self):
        request = GetWaypoints.Request()
        request.get_waypoints = True
        return self.cli.call_async(request)
    
    def receive_goals(self):
        future = self.send_request()
        # Execute the service until the future is complete
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info('Waypoints received by the action client')

            for pose in response.waypoints.poses:
                waypoint = [pose.position.x, pose.position.y, pose.position.z]
                self.goals.append(waypoint)
                self.get_logger().info(f'Waypoint: {waypoint}')
            
            if self.goals:
                self.send_goal(self.goals[0])
            else:
                self.get_logger().error('No waypoints received!')
        else:
            self.get_logger().error('Service call failed %r' % (future.exception(),))

def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    waypoint_client.receive_goals()

    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
