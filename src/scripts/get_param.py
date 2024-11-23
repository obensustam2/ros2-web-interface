import rclpy
from rclpy.node import Node


class ParameterPrinter(Node):
    def __init__(self):
        super().__init__('fanuc_gripper_multi_group')

        # Declare the parameter with a default value
        self.declare_parameter('max_velocity_scaling_factor', 0.0)

        # Set up a timer to call the callback periodically
        self.timer = self.create_timer(1.0, self.print_parameter_value)  # 1.0 seconds

    def print_parameter_value(self):
        # Get the current value of the parameter
        param_value = self.get_parameter('max_velocity_scaling_factor').value

        # Print the parameter value
        self.get_logger().info(f"Parameter value: {param_value}")


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    parameter_printer = ParameterPrinter()
    rclpy.spin(parameter_printer)

    # Destroy the node and shutdown
    parameter_printer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
