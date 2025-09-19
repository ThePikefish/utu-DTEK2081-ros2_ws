import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_euler

class TFStaticBroadcaster(Node):
    """
    Broadcast a static transform from the 'robot' frame to the 'lidar' frame.
    The 'robot' frame moves in a circular path.
    """
    def __init__(self):
        super().__init__('tf_static_broadcaster')
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.static_broadcast()
        self.get_logger().info('Node started!')

    def static_broadcast(self):
        """
        Broadcast the static transform.
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'robot'     # Parent
        transform_stamped.child_frame_id = 'lidar'      # Child

        # Set translation
        transform_stamped.transform.translation.x = 0.5 # X forward
        transform_stamped.transform.translation.y = 0.0 # Y
        transform_stamped.transform.translation.z = 0.5 # Z up

        # Set rotation
        q = quaternion_from_euler(0.0, 0.0, 0.0)
        transform_stamped.transform.rotation.x = q[0]
        transform_stamped.transform.rotation.y = q[1]
        transform_stamped.transform.rotation.z = q[2]
        transform_stamped.transform.rotation.w = q[3]

        self.tf_static_broadcaster.sendTransform(transform_stamped)



def main(args=None):
    rclpy.init(args=args)
    tf_static_broadcaster = TFStaticBroadcaster()
    rclpy.spin(tf_static_broadcaster)
    tf_static_broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()