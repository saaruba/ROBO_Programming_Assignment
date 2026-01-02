import rclpy
from rclpy.node import Node

class InventoryNode(Node):
    def __init__(self):
        super().__init__('inventory_node')
        self.get_logger().info('Emergency Inventory Node is running')

def main(args=None):
    rclpy.init(args=args)
    node = InventoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

