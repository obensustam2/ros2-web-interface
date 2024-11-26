import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class MultiServiceServer(Node):
    def __init__(self):
        super().__init__('multi_service_server')

        self.get_logger().info('Service server initialized')

        # Create services for each command
        self.start_service = self.create_service(Empty, '/start', self.start_callback)
        self.stop_service = self.create_service(Empty, '/stop', self.stop_callback)
        self.pause_service = self.create_service(Empty, '/pause', self.pause_callback)
        self.home_service = self.create_service(Empty, '/home', self.home_callback)

    def start_callback(self, request, response):
        self.get_logger().info('Start service called')
        return response

    def stop_callback(self, request, response):
        self.get_logger().info('Stop service called')
        return response

    def pause_callback(self, request, response):
        self.get_logger().info('Pause service called')
        return response

    def home_callback(self, request, response):
        self.get_logger().info('Home service called')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MultiServiceServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
