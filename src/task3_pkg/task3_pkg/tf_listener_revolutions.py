import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

class TFListenerRevolutions(Node):
    """
    Listens for a transform from 'robot' to 'map' and logs the revolutions.
    """
    def __init__(self):
        super().__init__('tf_listener_revolutions')
        self.tf_buffer = Buffer()
        self.tf_listener_revolutions = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.timer_callback)

        self.last_yaw = None
        self.revolutions = 0

        self.get_logger().info('Node started!')

    def timer_callback(self):
        """
        Periodically looks up the transform and calculates revolutions from yaw.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'robot',
                rclpy.time.Time()
            )

            # Position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            q = (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )
            roll, pitch, yaw = euler_from_quaternion(q)
            #self.get_logger().info(f'Yaw: {yaw}')

            if self.last_yaw is not None:
                delta = yaw - self.last_yaw

                # Handle yaw going around
                if delta > math.pi:
                    delta -= 2 * math.pi
                elif delta < -math.pi:
                    delta += 2 * math.pi

                # Accumulate revolution count
                if delta > 0 and (self.last_yaw < 0 and yaw >= 0):
                    pass  # crossing zero
                if delta != 0:
                    self.total_yaw += delta
                    revolutions_now = int(self.total_yaw / (2 * math.pi))
                    if revolutions_now != self.revolutions:
                        self.revolutions = revolutions_now
                        self.get_logger().info(f'Revolutions: {self.revolutions}')
        
            else:
                # Initialize tracking of yaw
                self.total_yaw = 0.0

            self.last_yaw = yaw


        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')



def main(args=None):
    rclpy.init(args=args)
    tf_listener_revolutions = TFListenerRevolutions()
    rclpy.spin(tf_listener_revolutions)
    tf_listener_revolutions.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()