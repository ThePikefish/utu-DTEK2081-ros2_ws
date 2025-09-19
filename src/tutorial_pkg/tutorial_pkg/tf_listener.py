import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

class TFListener(Node):
    """
    Listens for a transform from 'map' to 'robot' and logs the yaw angle.
    """
    def __init__(self):
        super().__init__('tf_listener')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.timer_callback)
        
        self.get_logger().info('Node started!')

    def timer_callback(self):
        """
        Periodically looks up the transform and prints the yaw.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'robot',
                rclpy.time.Time()
            )

            q = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            roll, pitch, yaw = euler_from_quaternion(q)
            self.get_logger().info(f'Yaw: {yaw}')

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')

def main(args=None):
    rclpy.init(args=args)
    tf_listener = TFListener()
    rclpy.spin(tf_listener)
    tf_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()