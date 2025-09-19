import math
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

class TFListenerTransform(Node):
    """
    Listens for a transform from 'scanner' to 'map' and logs the distance and speed relative to map.
    """
    def __init__(self):
        super().__init__('tf_listener_transform')
        self.tf_buffer = Buffer()
        self.tf_listener_transform = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(2, self.timer_callback)

        self.last_position = None
        self.last_time = None

        self.get_logger().info('Node started!')

    def timer_callback(self):
        """
        Periodically looks up the transform and prints the distance and speed
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'scanner',
                rclpy.time.Time()
            )

            # Position
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # Distance from map origin
            distance = math.sqrt(x**2 + y**2 + z**2)
            self.get_logger().info(f'Distance from map origin: {distance} m')

            # Speed
            time = self.get_clock().now().seconds_nanoseconds()[0] + \
            self.get_clock().now().seconds_nanoseconds()[1] / 1e9

            position = (x, y, z)

            # Speed = distance / time
            if self.last_position is not None:
                dt = time - self.last_time
                dx = position[0] - self.last_position[0]
                dy = position[1] - self.last_position[1]
                dz = position[2] - self.last_position[2]

                speed = math.sqrt(dx**2 + dy**2 + dz**2) / dt
                self.get_logger().info(f'Scanner speed: {speed} m/s')

            self.last_position = position
            self.last_time = time

        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')



def main(args=None):
    rclpy.init(args=args)
    tf_listener_transform = TFListenerTransform()
    rclpy.spin(tf_listener_transform)
    tf_listener_transform.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()