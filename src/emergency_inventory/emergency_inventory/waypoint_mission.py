import time # Used to measure timeouts
from dataclasses import dataclass # For defining Way points
from typing import List, Optional # Fro readablity

import rclpy
from rclpy.node import Node

#PoseStamped = Nav2 goal format (position + orientation in a frame)
#Twist = velocity command for the recovery spin
from geometry_msgs.msg import PoseStamped, Twist 

#BasicNavigator = simple API to send goals to Nav2
#TaskResult = tells you if nav succeeded/failed/canceled
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


@dataclass
class Waypoint:
    x: float
    y: float
    qx: float
    qy: float
    qz: float
    qw: float

# thuis is for converting your simple numbers into a PoseStamped message
def make_pose(nav: BasicNavigator, wp: Waypoint, frame_id: str = "map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = float(wp.x)
    pose.pose.position.y = float(wp.y)
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = float(wp.qx)
    pose.pose.orientation.y = float(wp.qy)
    pose.pose.orientation.z = float(wp.qz)
    pose.pose.orientation.w = float(wp.qw)
    return pose


class WaypointMission(Node):
    """
    Robust:
    - go waypoint by waypoint (so we can timeout/skip)
    - return home with retries
    - clean shutdown (no rclpy.shutdown inside __init__)
    """

    def __init__(self):
        super().__init__("waypoint_mission")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.nav = BasicNavigator()
        self.get_logger().info("Waiting for Nav2...")
        self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 active.")

        #WAYPOINTS (keep your list here)
        INSPECTION_ROUTE_RAW = [
              [1.6731071472167969, 0.47867584228515625, 0.0, 0.0, 0.6903947382174741, 0.7234328617374424],
              [1.9149692058563232, 3.3921375274658203, 0.0, 0.0, -0.4507699941692315, 0.8926401359767949],
              [2.454200506210327, 5.068150520324707, 0.0, 0.0, -0.057747343393347424, 0.9983312297684626],
              [5.112896919250488, 4.934216499328613, 0.0, 0.0, -0.1434589095671225, 0.9896562743022509],
              [4.190258979797363, 2.6927971839904785, 0.0, 0.0, -0.07540651078823325, 0.9971528760078587],
              [4.891469955444336, 0.914207935333252, 0.0, 0.0, 0.9974265772788851, 0.07169534808987339],
              [4.6286420822143555, -1.5605735778808594, 0.0, 0.0, 0.9995276995137392, 0.03073073225245809],
              [7.0873236656188965, -1.5170783996582031, 0.0, 0.0, -0.6690762903429458, 0.7431937282438019],
              [4.493269920349121, -3.4815115928649902, 0.0, 0.0, 0.9992319741555286, 0.03918496937915112],
              [1.0374326705932617, -2.9660587310791016, 0.0, 0.0, 0.6901232624008747, 0.7236918423563813],
              [-1.2540099620819092, -2.257345199584961, 0.0, 0.0, -0.5329208687701358,  0.84616508296507],
              [-3.4083499908447266, -2.7400002479553223, 0.0, 0.0, -0.9215235621037589, 0.38832244911619473],
              [-2.3540353775024414, -0.1683100461959839, 0.0, 0.0,  0.9998612334378453, 0.01665874749645358],
              [-3.2466983795166016, 3.91264271736145, 0.0, 0.0,  -0.48184806166876937,  0.8762547834197825],
              [0.37414658069610596, 4.039180278778076, 0.0, 0.0,  0.9647689995502441,  0.2630984179101448],
              [1.7672016620635986, 2.649862289428711, 0.0, 0.0,  -0.7285637687888756,  0.6849779812579013],
              [1.879518985748291,  0.09494161605834961, 0.0, 0.0,  0.007748276598128194,  0.9999699816543289],
              [0.03510642051696777, 0.11967086791992188, 0.0, 0.0,  0.001097844184200295,  0.999999397368892]

        ]
        

        if len(INSPECTION_ROUTE_RAW) < 2:
            raise RuntimeError("INSPECTION_ROUTE_RAW must contain at least 2 waypoints.")

        # This converts each list into a Waypoint object
        self.route: List[Waypoint] = [Waypoint(*w) for w in INSPECTION_ROUTE_RAW]

        # first waypoint
        self.home: Waypoint = self.route[0]

        self.frame_id = "map" # This is to match the  waypoint frame (amcl_pose)
        self.wp_timeout_s = 60.0
        self.return_timeout_s = 90.0
        self.return_retries = 2 # If return fails then try again after a recovery spin

        self.i = 0
        self.phase = "GO_WAYPOINTS"
        self.current_start_t: Optional[float] = None # Time when current goal started
        self._return_attempt = 0 # Counts retry attempts when it returning home.

        self.get_logger().info(f"Starting mission with {len(self.route)} waypoints.")

        self._start_next_waypoint() # Start mission and timer
        self.timer = self.create_timer(0.2, self._tick)

    def _start_next_waypoint(self): # this function, Start next waypoint mission 
        if self.i >= len(self.route): #If all waypoints are done → switch to return home.
            self.phase = "RETURN_HOME"
            self.current_start_t = None
            self.get_logger().info("All waypoints attempted. Switching to RETURN_HOME.")
            self._start_return_home()
            return

        wp = self.route[self.i]
        pose = make_pose(self.nav, wp, frame_id=self.frame_id)
        self.get_logger().info(f"Going to waypoint {self.i}: ({wp.x:.2f}, {wp.y:.2f})")
        self.nav.goToPose(pose)
        self.current_start_t = time.time()

    def _start_return_home(self):
        home_pose = make_pose(self.nav, self.home, frame_id=self.frame_id)
        self.get_logger().info(f"Returning home: ({self.home.x:.2f}, {self.home.y:.2f})")
        self.nav.goToPose(home_pose)
        self.current_start_t = time.time()

    def _recovery_spin(self, seconds: float = 2.0):
        end_t = time.time() + seconds
        cmd = Twist()
        cmd.angular.z = 0.8
        while time.time() < end_t and rclpy.ok():
            self.cmd_pub.publish(cmd)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())

    def _tick(self): # the main mission brain
        if self.phase == "DONE":
            return
        
        # Then check timeout
        if not self.nav.isTaskComplete(): 
            if self.current_start_t is not None:
                # if stuck too long, then cancel and later skip or retry
                elapsed = time.time() - self.current_start_t
                limit = self.wp_timeout_s if self.phase == "GO_WAYPOINTS" else self.return_timeout_s
                if elapsed > limit:
                    self.get_logger().warn(f"Timeout in phase {self.phase}. Canceling task.")
                    try:
                        self.nav.cancelTask()
                    except Exception:
                        pass
            return

        result = self.nav.getResult()

        if self.phase == "GO_WAYPOINTS":
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Waypoint {self.i} reached.")
            else:
                self.get_logger().warn(f"Waypoint {self.i} not reached (status={result}). Skipping.")
            self.i += 1
            self._start_next_waypoint()
            return
        
        # If success set DONE and shutdown
        if self.phase == "RETURN_HOME":
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Returned home successfully.")
                self.phase = "DONE"
                self.get_logger().info("Mission complete. Shutting down.")
                rclpy.shutdown()
                return

            self._return_attempt += 1
            self.get_logger().warn(f"Return home failed (attempt {self._return_attempt}).")

            if self._return_attempt <= self.return_retries:
                self._recovery_spin(seconds=2.0)
                self._start_return_home()
            else:
                self.get_logger().warn("Could not return home after retries. Ending mission.")
                self.phase = "DONE"
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()




    