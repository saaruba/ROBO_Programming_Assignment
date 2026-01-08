import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from sensor_msgs.msg import Image, LaserScan
import time


class EmergencyInventoryNode(Node):

    def __init__(self):
        super().__init__('inventory_node')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)


        self.image_sub = self.create_subscription(
            Image,
            '/limo/depth_camera_link/image_raw',
            self.image_callback,
            10
        )


        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # STATE the MACHINE
        self.state = "FORWARD"
        self.state_start_time = time.time()

        self.get_logger().info('Emergency Inventory Node with STATE MACHINE started with BACK-CHECK started.')

    def image_callback(self, msg):
        pass  # will be used later for inventory

    def change_state(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f"STATE → {new_state}")

    def scan_callback(self, msg):

        ranges = [
            r if 0.1 < r < msg.range_max else msg.range_max
            for r in msg.ranges
        ]

        mid = len(ranges) // 2

        front = min(ranges[mid - 10: mid + 10])
        left  = min(ranges[mid + 30: mid + 70])
        right = min(ranges[mid - 70: mid - 30])
        back  = min(ranges[mid + 170: mid - 170:-1])

        cmd = Twist()
        now = time.time()

        # Got stuck got Boxed
        boxed_in = front < 0.5 and left < 0.5 and right < 0.5

        # Turning Logics 

        if self.state == "FORWARD":

            if boxed_in:
                self.change_state("CHECK_BACK")

            elif front > 0.6:
                cmd.linear.x = 0.2

            else:
                self.change_state("TURNING")

        elif self.state == "TURNING":

            cmd.angular.z = 0.8 if left > right else -0.8

            if now - self.state_start_time > 1.2:
                self.change_state("ESCAPE")

        elif self.state == "ESCAPE":

            cmd.linear.x = 0.15
            cmd.angular.z = 0.3

            if now - self.state_start_time > 1.5:
                self.change_state("FORWARD")

        elif self.state == "CHECK_BACK":

            # rotate 180 degrees
            cmd.angular.z = 1.0

            if now - self.state_start_time > 2.8:
                if back > 0.8:
                    self.get_logger().info("Back is clear → reversing direction")
                    self.change_state("FORWARD")
                else:
                    self.get_logger().info("Back blocked → forced escape")
                    self.change_state("ESCAPE")

        self.cmd_pub.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = EmergencyInventoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()

