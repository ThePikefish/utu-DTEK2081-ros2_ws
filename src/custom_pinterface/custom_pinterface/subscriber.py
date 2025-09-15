import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from custom_interface.msg import Person
from rclpy.time import Time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('subscriber')
        topic_name = 'PersonTopic' # DONE: # subscribe to the publisher’s topic.
        
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keep messages even if the publisher dies
            history=HistoryPolicy.KEEP_LAST,                # Only store the last N messages
            depth=10                                        # The number of messages to store (N=10)
        )

        self.subscription = self.create_subscription(
            Person,
            topic_name,
            self.listener_callback,
            self.qos_profile)
        
        self.subscription
        
        self.prev = None


    def listener_callback(self, msg):
        self.get_logger().info(f'Received Person: Name= {msg.name}, Age= {msg.age}, Student= {msg.is_student}')
        
        current_time = Time.from_msg(msg.header.stamp)
        if self.prev is not None:
            dt = (current_time - self.prev).nanoseconds * 1e-9 
            freq = 1.0 / dt
            self.get_logger().info(f"dt={dt:.3f}s -> freq={freq:.2f} Hz")
        self.prev = current_time



def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
