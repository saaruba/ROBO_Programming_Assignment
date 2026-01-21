import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


@dataclass
class Waypoint:
    x: float
    y: float
    qx: float
    qy: float
    qz: float
    qw: float


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
              [0.4380440960076984, -0.30017426665114455, 0.0, 0.0, 0.3180586709926065, 0.9480710320468698],
              [1.2917316552750913, 1.3395856515052869, 0.0, 0.0, 0.5593811599339845, 0.8289105608634203],
              [1.5137642299622605, 3.7317498348382148, 0.0, 0.0, 0.8245023571578856, 0.5658585185725231],
              [-1.4596853809263386, 4.351749368545111, 0.0, 0.0, -0.9495298982077545, 0.3136765410571396],
              [-3.511686499464037, 3.6835452723832023, 0.0, 0.0, -0.891254421912406, 0.4535036443310936],
              [-3.7600257744766648, 0.026575861573809072, 0.0, 0.0, -0.7181551417321504, 0.6958830306909882],
              [-4.395047713571779, -2.3263015888048595, 0.0, 0.0, -0.9915982884981513, 0.12935545696852882],
              [-7.449072207369064, -2.8823181476311213, 0.0, 0.0, 0.8219698017509366, 0.5695310746654005],
              [-7.906326019694105, -0.9245097417426785, 0.0, 0.0, 0.6415269502258909, 0.7671004967628865],
              [-7.026622723663798, 0.34973345830635066, 0.0, 0.0, 0.23995693446932348, 0.9707835338529824],
              [-3.912056464922836, 1.6556356976778395, 0.0, 0.0, 0.4207351905551278, 0.9071834982122086],
              [-2.027907414118563, 4.12780583771418, 0.0, 0.0, -0.48735511374436047, 0.8732038668645608],
              [-1.2221214449184898, -3.3551669394422463, 0.0, 0.0, -0.8819189081205386, 0.9975891122311885],
              [1.704894046754486, -3.363856041126271, 0.0, 0.0, -0.06939714084736646, -0.8819189081205386],
              [-1.5583339546094246, -5.244009840714862, 0.0, 0.0, -0.9742772592720974, 0.2253526615446337],
              [-4.345883403876832, -4.788514179417724, 0.0, 0.0, 0.1858277962438677, 0.9825823274123892],
              [-1.569044845369205, -7.161639222881427, 0.0, 0.0, -0.9866277838434035, 0.1629896197561493],
              [-6.832440742091754, -6.939090311729447, 0.0, 0.0, 0.875122754759805, 0.4839009858448421],
              [-6.6257464391661935, -4.771148787468491, 0.0, 0.0, -0.4849353876442543, 0.8745499813105692],
              [-1.3330199215404106, -7.288032461954166, 0.0, 0.0, 0.10677527324132809, 0.9942831794937697],
              [5.4406620484181705, -6.055436624779557, 0.0, 0.0, 0.9050011303013623, 0.42540916087133873],
              [5.344888358338532, -2.385667227254904, 0.0, 0.0, 0.30021416140464807, 0.9538718243517332],
              [5.031826573347964, 3.478023304748386, 0.0, 0.0, 0.29311597804087214, 0.9560768920003991],
              [8.961607670217626, 4.792547380662433, 0.0, 0.0, -0.5654237402385451, 0.8248005783064499],
              [7.500587747689686, 0.6642534281526508, 0.0, 0.0, -0.6130819825765111, 0.7900192925745892],
              [7.910669990913888, -8.000716869076049, 0.0, 0.0, 0.8584421660848659, 0.5129103698381653],
              [5.180036198503661, 2.761183283110034, 0.0, 0.0, -0.8907959733836289, 0.45440349228798077],
              [6.071865837038928, -5.8038085191674735, 0.0, 0.0, -0.8823300656069174, 0.47063112447658284],
              [0.6231136396647378, -7.572336410229179, 0.0, 0.0, 0.9355388338293172, 0.35322385309783544],
              [-1.8457560138700526, -1.2581382408655117, 0.0, 0.0, 0.7227624185803786, 0.6910965824599639],
              [-0.22022726168401346, 3.0054245006567957, 0.0, 0.0, 0.5407743433633871, 0.8411677059658779],
              [1.516335030113087, 1.2285070618712433, 0.0, 0.0, -0.8958543819991882, 0.44434775373895236],
              [0.16668358796585533, 0.13621706608333062, 0.0, 0.0, -0.1711037268864201, 0.9852530206224072]


        ]
        

        if len(INSPECTION_ROUTE_RAW) < 2:
            raise RuntimeError("INSPECTION_ROUTE_RAW must contain at least 2 waypoints.")

        self.route: List[Waypoint] = [Waypoint(*w) for w in INSPECTION_ROUTE_RAW]
        self.home: Waypoint = self.route[0]

        self.frame_id = "map"
        self.wp_timeout_s = 60.0
        self.return_timeout_s = 90.0
        self.return_retries = 2

        self.i = 0
        self.phase = "GO_WAYPOINTS"
        self.current_start_t: Optional[float] = None
        self._return_attempt = 0

        self.get_logger().info(f"Starting mission with {len(self.route)} waypoints.")
        self._start_next_waypoint()
        self.timer = self.create_timer(0.2, self._tick)

    def _start_next_waypoint(self):
        if self.i >= len(self.route):
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

    def _tick(self):
        if self.phase == "DONE":
            return

        if not self.nav.isTaskComplete():
            if self.current_start_t is not None:
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