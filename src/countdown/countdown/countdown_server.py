import rclpy
from rclpy.node import Node
from custom_interface.action import Countdown
from rclpy.action import ActionServer
import time

class CountdownServerNode(Node):

    def __init__(self):
        super().__init__('countdown_server_node')

        self.action_server_ = ActionServer(
            self, 
            Countdown, 
            'countdown_action', # DONE: fill with the name of the action 
            execute_callback=self.execute_callback
        )

        self.get_logger().info('countdown action server has been started')

    
    def execute_callback(self, goal_handle):
        """execute the action and return the result"""

        self.get_logger().info('executing the goal')

        # get goal request value
        start_from = goal_handle.request.start_from
        self.get_logger().info(f"Counting down from {start_from}")

        # initialize feedback
        feedback_msg = Countdown.Feedback()
        current = start_from

        # process Countdown
        while current >= 0:
            feedback_msg.current = current 
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Feedback: current number={current}")
            time.sleep(1.0)
            current -= 1
        
        self.get_logger().info("Countdown completed!")
        
        # set the goal state as succeeded
        goal_handle.succeed()

        # return the result
        result = Countdown.Result()
        result.result_text = f'Counted down from {start_from} to 0'

        return result


def main(args=None):
    rclpy.init(args=args)
    node = CountdownServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=='__main__':
    main()