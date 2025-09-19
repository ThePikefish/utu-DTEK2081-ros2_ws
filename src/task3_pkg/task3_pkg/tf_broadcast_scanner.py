import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class TFBroadcastScanner(Node):
    """
    Broadcasts a transform from the 'robot' frame to the 'scanner' frame.
    The 'scanner' frame oscillates.
    """
    def __init__(self):
        super().__init__('tf_broadcast_scanner')
        self.tf_broadcast_scanner = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.frequency = 1.0 # Oscillation frequency in Hz

        self.get_logger().info('Node started!')

    def timer_callback(self):
        """
        Periodically broadcasts the transform.
        """
        t = self.get_clock().now().seconds_nanoseconds()[0] + \
            self.get_clock().now().seconds_nanoseconds()[1] / 1e9

        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'robot'     # Parent
        transform_stamped.child_frame_id = 'scanner'    # Child

        # Set translation
        transform_stamped.transform.translation.x = 0.5
        transform_stamped.transform.translation.y = 0.3 * math.sin(2 * math.pi * self.frequency * t)
        transform_stamped.transform.translation.z = 0.7

        # Set rotation
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.tf_broadcast_scanner.sendTransform(transform_stamped)

def main(args=None):
    rclpy.init(args=args)
    tf_broadcast_scanner = TFBroadcastScanner()
    rclpy.spin(tf_broadcast_scanner)
    tf_broadcast_scanner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()