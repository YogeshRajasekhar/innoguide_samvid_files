#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import subprocess


class UserInputStarterNode(Node):
    def __init__(self):
        super().__init__('user_input_starter_node')
        

    def start_another_launch(self, node_name, package_name):
        """Starts another ROS2 node."""
        try:
            self.get_logger().info(f"Starting node '{node_name}' from package '{package_name}'...")
            subprocess.Popen(['ros2', 'launch', package_name, node_name])
        except Exception as e:
            self.get_logger().error(f"Failed to start node '{node_name}': {e}")

    def get_user_input(self):
        """Waits for user input to trigger the node start."""
        while rclpy.ok():
            user_input = input("Press 'Y' to start the other node or 'Q' to quit: ").strip().upper()
            if user_input == "Y":
                # Example usage: Start the target node
                self.start_another_launch('turtlebot3_world.launch.py','turtlebot3_gazebo')
            elif user_input == "Q":
                self.get_logger().info("Exiting UserInputStarterNode.")
                break
            else:
                print("Invalid input. Please press 'Y' to start or 'Q' to quit.")


def main(args=None):
    rclpy.init(args=args)
    node = UserInputStarterNode()

    # Run the user input loop
    try:
        node.get_user_input()
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted. Shutting down.")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
