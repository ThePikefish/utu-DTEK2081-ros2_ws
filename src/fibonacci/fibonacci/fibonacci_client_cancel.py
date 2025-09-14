import rclpy
from rclpy.node import Node
from custom_interface.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

import time

class FibonacciCancelClientNode(Node):
    def __init__(self):
        super().__init__('fibonacci_cancel_client_node')

        self.action_client_ = ActionClient(self, Fibonacci, 'fibonacci_cancel') # TODO: fill with action name

        # get goal parameter value
        # TODO: declare and set ROS parameter varlue for fibonacci order
        self.declare_parameter('order', 5) # default 5
        self.order_ = self.get_parameter('order').value

        self.get_logger().info('Fibonacci action client has been started')

    def send_goal(self):
        """ sends goal request """

        self.get_logger().info('Waiting for action server...')
        self.action_client_.wait_for_server()


        # create a goal
        goal = Fibonacci.Goal()
        goal.order = self.order_

        self.get_logger().info('Sending goal request...')

        self.action_client_.send_goal_async(goal,feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)


    def feedback_callback(self, feedback):
        """ logs feedback response """
        self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.partial_sequence))


    def goal_response_callback(self, future):
        """ if the goal is accepted, starts a timer. After timeout, timer_callback gets called"""

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')

        # Start a 2 second timer
        self._timer = self.create_timer(2.0, self.timer_cancel_goal_callback) #  TODO: add time for when to cancel, consider the timing from the server

    
    def timer_cancel_goal_callback(self):
        """ sends cancel goal async request"""

        self.get_logger().info('fibonacci action server has been canceled!') # TODO: Assing a meessage for the cancelation
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async().add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()


    def cancel_done(self, future):
        """ receives cancel goal response and checks if it was successfull"""

        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal sucessfully canceled') # TODO: message for the goal sucessfully canceled
        else:
            self.get_logger().info('Goal failed to cancel') # TODO: message for the goal failed to cancel

        rclpy.shutdown()

    
def main(args=None):
    rclpy.init(args=args)

    node = FibonacciCancelClientNode()
    node.send_goal()

    rclpy.spin(node)
    rclpy.try_shutdown()

if __name__=='__main__':
    main()