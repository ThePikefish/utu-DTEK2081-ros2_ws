import rclpy
from rclpy.node import Node
from custom_interface.srv import CalculateDistance
from geometry_msgs.msg import Point
from functools import partial

class DistanceClientNode(Node):

    def __init__(self):
        super().__init__('calculate_distance_client_node')
        self.get_logger().info('Client Python node has been created')

        # declare parameters
        self.declare_parameter('point1', [0.0, 0.0])
        self.declare_parameter('point2', [0.0, 0.0])

        # get parameters
        point1_ = Point()
        point1_.x = self.get_parameter('point1').value[0]
        point1_.y = self.get_parameter('point1').value[1]

        point2_ = Point()
        point2_.x = self.get_parameter('point2').value[0]
        point2_.y = self.get_parameter('point2').value[1]
        
        
        self.call_distance_server(point1_, point2_)

    
    def call_distance_server(self, point1, point2):
        client = self.create_client(CalculateDistance, 'calculate_distance_service') # DONE: fill with your service name
        while not client.wait_for_service(1.0):
            self.get_logger().info('Waiting for the Server...')

        # create request
        request = CalculateDistance.Request()
        request.point1 = point1 # DONE: asign the correct variable
        request.point2 = point2 # DONE: asign the correct variable

        # send request asynchronously
        future = client.call_async(request)
        future.add_done_callback(partial(self.distance_service_callback, point1=point1, point2=point2))


    def distance_service_callback(self, future, point1, point2):
        try:
            response = future.result()
            if response.success == True:
                self.get_logger().info(f'Distance between {point1} and {point2} is {response.distance}. {response.message}') # DONE: print the values and the result
            else:
                self.get_logger().info(f'Calculation failed. {response.message}')
        except Exception as e:
            self.get_logger().info(f'Service call failed {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DistanceClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=='__main__':
    main()