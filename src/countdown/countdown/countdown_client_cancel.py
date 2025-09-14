import rclpy
from rclpy.node import Node
from custom_interface.action import Countdown
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

import time

class CountdownClientNode(Node):

    def __init__(self):
        super().__init__('countdown_client_node_cancel')

        self.action_client_ = ActionClient(
            self, 
            Countdown, 
            'countdown_action' # TODO: fill with action name
        )


        self.declare_parameter('start_from', 10) # default 10
        self.start_from = self.get_parameter('start_from').value
        
        self.declare_parameter('cancel_after', 2.0)
        self.cancel_after = float(self.get_parameter('cancel_after').value)
    

        self.get_logger().info('Countdown action client has been started')


    def send_goal(self, start_from):
        """the action client sends the goal"""

        # wait for the server
        timeout_ = 1.0
        self.action_client_.wait_for_server(timeout_sec=timeout_)

        # create a goal
        goal = Countdown.Goal()
        goal.start_from = start_from

        # send the goal
        self.get_logger().info(f'sending the goal: {start_from}')

        # send_goal_async with feedback_callback. It returns future. when future is returned, we can add a callback
        self.action_client_.send_goal_async(goal, feedback_callback=self.goal_feedback_callback).add_done_callback(self.goal_response_callback)

    
    def goal_feedback_callback(self, feedback_msg):
        """gets feedback message from feedback_callback"""

        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: current={feedback.current}')


    def goal_response_callback(self, future):
        """checks if the goal request was accepted"""

        goal_handle = future.result()

        # checks if the goal has been accepted
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected ')
            return 
        
        
        self._goal_handle = goal_handle
        self.get_logger().info('Goal accepted :)')

        # Start a timer
        self._timer = self.create_timer(self.cancel_after, self.timer_cancel_goal_callback)



    def timer_cancel_goal_callback(self):
        """cancels the goal after a certain time"""
        self.get_logger().info('countdown action server has been canceled!')
        
        future = self._goal_handle.cancel_goal_async().add_done_callback(self.cancel_done)
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
    node = CountdownClientNode()

    # send goal
    node.send_goal(node.start_from)

    rclpy.spin(node)

    try:
        rclpy.shutdown()
    except:
        pass
        

if __name__=='__main__':
    main()
