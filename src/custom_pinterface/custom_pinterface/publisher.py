import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from custom_interface.msg import Person

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        topic_name = 'PersonTopic' # DONE: fill with the name of the topic

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keep messages even if the publisher dies
            history=HistoryPolicy.KEEP_LAST,                # Only store the last N messages
            depth=10                                        # The number of messages to store (N=10)
        )

        self.publisher_ = self.create_publisher(
            Person,
            topic_name,
            self.qos_profile
            )
        
        timer_period = 1 # DONE:      # time in seconds
        self.timer = self.create_timer(timer_period,
                                        self.timer_callback)

        self.i = 0


    def timer_callback(self):
        msg = Person()
        
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.name = "Hauki"
        msg.age = 18
        msg.is_student = True

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Person: {msg.name}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
