import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image, LaserScan


class EmergencyInventoryNode(Node):

    def __init__(self):
        super().__init__('inventory_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to camera images
        self.image_sub = self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to laser scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.get_logger().info('Emergency Inventory Node started. Listening to sensors...')

    def image_callback(self, msg):
        self.get_logger().info(
            f'Received camera image: width={msg.width}, height={msg.height}'
        )

    def scan_callback(self, msg):
        mid = len(msg.ranges) // 2
        front_ranges = msg.ranges[mid - 10 : mid + 10]

        # Remove zero / invalid values
        front_ranges = [r for r in front_ranges if r > 0.0]

        if not front_ranges:
            return
        
        front_distance = min(front_ranges)

        cmd = Twist()

        if front_distance > 0.35:
            cmd.linear.x = 0.1
            cmd.angular.z = 0.0
            self.get_logger().info('Moving forward')

        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.0     # turn
            self.get_logger().info('Obstacle detected! Turning')
        
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyInventoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

