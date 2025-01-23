################################################### DO NOT MODIFY ##############################################################

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from my_robot_interfaces.action import MoveCircle 
from geometry_msgs.msg import Twist
import math
import time


class TurtleCircleActionServer(Node):

    def __init__(self):
        super().__init__('turtle_circle_action_server')
        self._action_server = ActionServer(
            self,
            MoveCircle,
            'move_circle',
            self.execute_callback
        )
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_period = 0.1 

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: radius={goal_handle.request.radius}, speed={goal_handle.request.speed}')
        radius = goal_handle.request.radius
        speed = goal_handle.request.speed

        # Initialize variables
        total_distance = 0.0
        distance_traveled = 0.0
        circumference = 2 * math.pi * radius
        angular_speed = speed / radius
        feedback_msg = MoveCircle.Feedback()

        # Start moving the turtle
        twist_msg = Twist()
        twist_msg.linear.x = speed
        twist_msg.angular.z = angular_speed

        start_time = time.time()
        while total_distance < circumference:

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                goal_handle.canceled()
                return MoveCircle.Result()

            self.publisher.publish(twist_msg)

            elapsed_time = time.time() - start_time
            distance_traveled = speed * elapsed_time
            feedback_msg.distance_traveled = distance_traveled
            goal_handle.publish_feedback(feedback_msg)

            total_distance = distance_traveled
            time.sleep(self.timer_period)

        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

        goal_handle.succeed()

        result = MoveCircle.Result()
        result.total_distance = total_distance
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = TurtleCircleActionServer()
    rclpy.spin(action_server)


if __name__ == '__main__':
    main()
########################################################################################################################################