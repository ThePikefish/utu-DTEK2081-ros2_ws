import rclpy
from rclpy.node import Node
from custom_interface.srv import CalculateDistance
from geometry_msgs.msg import Point
import math

class DistanceServerNode(Node):

    def __init__(self):
        super().__init__("calculate_distance_server")
        self.server_ = self.create_service(CalculateDistance, "calculate_distance_service", self.calculate_distance_service_callback) # TODO: give your sevice a name
        self.get_logger().info("Service server Python node has been created")
    

    def calculate_distance_service_callback(self, request, response):
        # Calculate euclidean distance
        dx = request.point2.x - request.point1.x
        dy = request.point2.y - request.point1.y
        distance = math.sqrt(dx*dx + dy*dy)

        if distance != None:
            response.distance = distance
            response.success = True
        else:
            response.success = False

        response.message = f'strig message'
        self.get_logger().info(f'Processed request: Point1={request.point1}, Point2={request.point2} -> {response.distance}, {response.success}, {response.message}')
        return response 


def main(args = None):
    rclpy.init(args=args)
    node = DistanceServerNode()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=='__main__':
    main()