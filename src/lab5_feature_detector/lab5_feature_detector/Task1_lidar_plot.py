import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt


class LidarPlotter(Node):
    def __init__(self):
        super().__init__('lidar_plotter')

        # Fix Qos profile
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1) 

        # Subscriptions
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        # subscribe to both new created topics (/feature_scan and /front_scan)
        self.feature_scan_subscription = self.create_subscription(
            LaserScan,
            '/feature_scan',
            self.feature_scan_callback,
            qos_profile
        )
        self.front_scan_subscription = self.create_subscription(
            LaserScan,
            '/front_scan',
            self.front_scan_callback,
            qos_profile
        )

        self.scan_data = None
        self.feature_scan_data = None
        self.front_scan_data = None

        # Matplotlib setup
        plt.ion()
        self.fig = plt.figure(figsize=(14, 7))
        self.ax_cart = self.fig.add_subplot(1, 2, 1)
        self.ax_polar = self.fig.add_subplot(1, 2, 2, projection='polar')
        self.get_logger().info("Matplotlib plot initialized...")

    def polar_to_cartesian_coordinate(self, ranges, angle_min, angle_max, angle_increment):
        """Convert LaserScan polar data to Cartesian coordinates."""
        # please complete this function to return required values
        
        # Convert to numpy array
        ranges = np.array(ranges)
        
        # remember to clip invalid values (inf, nan), if needed
        # Replace 0.0 with nan, as other node uses 0.0 for filtered points
        ranges[np.isinf(ranges)] = np.nan
        ranges[np.isnan(ranges)] = np.nan
        ranges[ranges == 0.0] = np.nan
        
        # Create an array of angles
        angles = np.linspace(angle_min, angle_max, len(ranges))
        
        # Convert to cartesian
        x_cart = ranges * np.cos(angles)
        y_cart = ranges * np.sin(angles)

        return angles, ranges, x_cart, y_cart

    def scan_callback(self, msg: LaserScan):
        self.scan_data = msg
        self.update_plot()

    # write two similar callback functions for the other two topics
    def feature_scan_callback(self, msg: LaserScan):
        self.feature_scan_data = msg
        self.update_plot()

    def front_scan_callback(self, msg: LaserScan):
        self.front_scan_data = msg
        self.update_plot()

    def update_plot(self):
        
        # please complete this function to plot the points

        # Clear both axes for the new plot
        self.ax_cart.clear()
        self.ax_polar.clear()
        
        # Data to plot: (data, color, label)
        plot_data = [
            (self.scan_data, 'blue', 'Full Scan'),
            (self.feature_scan_data, 'green', 'Feature Scan (1-2.5m)'),
            (self.front_scan_data, 'red', 'Front Scan (-60 to +60 deg)')
        ]
        
        # Plot all three topics if data is available
        for data, color, label in plot_data:
            if data is not None:
                # Convert data to plot-friendly format
                angles, ranges, x, y = self.polar_to_cartesian_coordinate(
                    data.ranges, data.angle_min, data.angle_max, data.angle_increment
                )
                
                # Plot to Cartesian
                self.ax_cart.scatter(x, y, s=1, c=color, label=label)
                
                # Plot to Polar
                self.ax_polar.scatter(angles, ranges, s=1, c=color, label=label)

        # Set properties for Cartesian plot
        self.ax_cart.set_title('Cartesian Lidar Scans')
        self.ax_cart.set_xlabel('X (m)')
        self.ax_cart.set_ylabel('Y (m)')
        self.ax_cart.set_aspect('equal')
        self.ax_cart.legend(markerscale=5)
        self.ax_cart.grid(True)
        self.ax_cart.set_xlim(-5, 5) # Set fixed limits
        self.ax_cart.set_ylim(-5, 5)

        # Set properties for Polar plot
        self.ax_polar.set_title('Polar Lidar Scans')
        self.ax_polar.set_rlim(0, 5) # Set radius limit
        self.ax_polar.set_theta_zero_location('N') # 0 degrees at top
        self.ax_polar.set_theta_direction(-1) # Clockwise
        self.ax_polar.legend(markerscale=5)
        
        # Redraw the canvas
        plt.draw()
        plt.pause(0.001) # Small pause to allow GUI to update


def main(args=None):
    rclpy.init(args=args)
    node = LidarPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
