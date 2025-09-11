import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('publisher')
        topic_name = 'Pose' # DONE: fill with the name of the topic

        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,         # Ensure delivery of messages
            durability=DurabilityPolicy.TRANSIENT_LOCAL,    # Keep messages even if the publisher dies
            history=HistoryPolicy.KEEP_LAST,                # Only store the last N messages
            depth=10                                        # The number of messages to store (N=10)
        )

        self.publisher_ = self.create_publisher(
            PoseStamped,
            topic_name,
            self.qos_profile
            )
        
        timer_period = 1 # DONE:      # time in seconds
        self.timer = self.create_timer(timer_period,
                                        self.timer_callback)

        self.step = 0
        self.direction = (1.0, 0.0, 0.0) # Moving direction
        self.speed = 0.1


    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.pose.position.x = self.direction[0] * self.speed * self.step
        msg.pose.position.y = self.direction[1] * self.speed * self.step
        msg.pose.position.z = self.direction[2] * self.speed * self.step

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: x={msg.pose.position.x}, y={msg.pose.position.y}')
        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
