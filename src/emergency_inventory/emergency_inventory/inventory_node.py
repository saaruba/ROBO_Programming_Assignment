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
        ranges = [
            r if 0.05 < r < msg.range_max else msg.range_max
            for r in msg.ranges
        ]
        
        mid = len(ranges) // 2
        
        front = min(ranges[mid - 10: mid + 10])
        left  = min(ranges[mid + 30: mid + 60])
        right = min(ranges[mid - 60: mid - 30])
        
        cmd = Twist()

    # Decision logic
        if front > 0.5:
            cmd.linear.x = 0.15
            cmd.angular.z = 0.0
            self.get_logger().info("Path clear → Moving forward")

        else:
            cmd.linear.x = 0.0

            if left > right:
                cmd.angular.z = 0.8
                self.get_logger().info("Obstacle → Turning LEFT")
            
            else:
                cmd.angular.z = -0.8
                self.get_logger().info("Obstacle → Turning RIGHT")

        self.cmd_pub.publish(cmd)


def main(args=None):
        rclpy.init(args=args)
        node = EmergencyInventoryNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
        main()    

    

 
