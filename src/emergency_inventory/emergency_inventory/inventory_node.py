import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32
from geometry_msgs.msg import PoseWithCovarianceStamped

import cv2
import numpy as np
from cv_bridge import CvBridge


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class InventoryDetector(Node):
    """
    Colour + shape detector 

    Objects (names you can use in your report):
      - fire_extinguisher  (red, tall cylinder-like silhouette)
      - first_aid_kit      (green, box-like silhouette)
      - aed_kit            (blue, box-like silhouette)

    De-dup strategy:
      - Use robot pose in map frame (amcl_pose) and a grid cell key.
      - Also merge neighbour cells within a radius in map to reduce re-counting.
    """

    def __init__(self):
        super().__init__("inventory_detector")

        # Topics (simulation topic as per workshops)
        self.image_topic = "/limo/depth_camera_link/image_raw"
        self.pose_topic = "/amcl_pose"  # map-frame pose (best for your mapped world)

        self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, self.pose_topic, self.pose_cb, 10)

        # Optional: publish polygon of detected blob in image coords (like workshop mentions)
        self.poly_pub = self.create_publisher(PolygonStamped, "/object_polygon", 10)

        self.bridge = CvBridge()

        # Current robot pose in map
        self.have_pose = False
        self.x = 0.0
        self.y = 0.0

        # De-dup parameters
        self.grid_size = 0.75  # meters
        self.dedup_radius_m = 1.0  # merge detections close to previous ones

        # Store detections as: item_type -> list of (x,y) detection anchors in map
        self.detection_anchors: Dict[str, List[Tuple[float, float]]] = {
            "fire_extinguisher": [],
            "first_aid_kit": [],
            "aed_kit": [],
        }

        # Also store grid keys to avoid fast repeats
        self.seen_cells = set()  # (cx, cy, item_type)

        # Output file
        self.out_path = Path(
            "/workspaces/cmp9767-ws-main/src/emergency_inventory/emergency_inventory/results.json"
        )

        # Debug
        self.show_debug = True
        self.debug_period = 0.25
        self._last_show = 0.0

        # HSV ranges (tune if needed)
        # Red needs wrap-around in HSV.
        self.hsv_ranges = {
            "fire_extinguisher_red": [
                ((0, 120, 70), (10, 255, 255)),
                ((170, 120, 70), (180, 255, 255)),
            ],
            "first_aid_kit_green": [
                ((40, 50, 50), (80, 255, 255)),
            ],
            "aed_kit_blue": [
                ((90, 50, 50), (130, 255, 255)),
            ],
        }

        # Minimum blob area (pixels)
        self.min_area = 1000

        # Shape heuristics (simple but effective for your 3 items)
        # - extinguisher tends to be taller than wide (aspect > ~1.2)
        # - kits are box-ish (aspect around ~0.6 to 1.8)
        self.extinguisher_min_aspect = 1.15
        self.box_min_aspect = 0.55
        self.box_max_aspect = 2.20
        self.min_extent = 0.35  # area / bounding_rect_area

        self.get_logger().info("InventoryDetector started.")

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.have_pose = True

    def _cell_key(self, item_type: str) -> Tuple[int, int, str]:
        cx = int(math.floor(self.x / self.grid_size))
        cy = int(math.floor(self.y / self.grid_size))
        return (cx, cy, item_type)

    def _is_duplicate(self, item_type: str) -> bool:
        # 1) Cell-based quick reject
        key = self._cell_key(item_type)
        if key in self.seen_cells:
            return True

        # 2) Distance-based merge (prevents recount across neighbouring cells)
        for ax, ay in self.detection_anchors[item_type]:
            if math.hypot(ax - self.x, ay - self.y) < self.dedup_radius_m:
                return True

        return False

    def _record_detection(self, item_type: str):
        key = self._cell_key(item_type)
        self.seen_cells.add(key)
        self.detection_anchors[item_type].append((self.x, self.y))

        self.get_logger().info(f"Detected {item_type} at map pose ({self.x:.2f}, {self.y:.2f}).")
        self.save_results()

    def counts(self) -> Dict[str, int]:
        return {k: len(v) for k, v in self.detection_anchors.items()}

    def save_results(self):
        data = {
            "timestamp_unix": time.time(),
            "map_frame_pose_source": self.pose_topic,
            "grid_size_m": self.grid_size,
            "dedup_radius_m": self.dedup_radius_m,
            "counts": self.counts(),
            "anchors": {
                k: [{"x": ax, "y": ay} for (ax, ay) in v]
                for k, v in self.detection_anchors.items()
            },
        }
        try:
            self.out_path.write_text(json.dumps(data, indent=2))
        except Exception as e:
            self.get_logger().warn(f"Could not write results.json: {e}")

    def _mask_from_ranges(self, hsv: np.ndarray, ranges) -> np.ndarray:
        mask = None
        for (lo, hi) in ranges:
            m = cv2.inRange(hsv, np.array(lo, dtype=np.uint8), np.array(hi, dtype=np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        return mask

    def _basic_cleanup(self, mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def _shape_ok(self, item_type: str, contour) -> bool:
        x, y, w, h = cv2.boundingRect(contour)
        if w <= 0 or h <= 0:
            return False

        area = cv2.contourArea(contour)
        rect_area = float(w * h)
        extent = area / rect_area

        aspect = float(h) / float(w)  # height/width

        if extent < self.min_extent:
            return False

        if item_type == "fire_extinguisher":
            return aspect >= self.extinguisher_min_aspect
        else:
            return (self.box_min_aspect <= aspect <= self.box_max_aspect)

    def _publish_polygon(self, contour, frame_id: str = "camera"):
        # Publish bounding rectangle as a simple polygon in image coordinates
        x, y, w, h = cv2.boundingRect(contour)
        poly = PolygonStamped()
        poly.header.stamp = self.get_clock().now().to_msg()
        poly.header.frame_id = frame_id
        poly.polygon.points = [
            Point32(x=float(x), y=float(y), z=0.0),
            Point32(x=float(x + w), y=float(y), z=0.0),
            Point32(x=float(x + w), y=float(y + h), z=0.0),
            Point32(x=float(x), y=float(y + h), z=0.0),
        ]
        self.poly_pub.publish(poly)

    def image_cb(self, msg: Image):
        if not self.have_pose:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build masks
        mask_red = self._mask_from_ranges(hsv, self.hsv_ranges["fire_extinguisher_red"])
        mask_green = self._mask_from_ranges(hsv, self.hsv_ranges["first_aid_kit_green"])
        mask_blue = self._mask_from_ranges(hsv, self.hsv_ranges["aed_kit_blue"])

        mask_red = self._basic_cleanup(mask_red)
        mask_green = self._basic_cleanup(mask_green)
        mask_blue = self._basic_cleanup(mask_blue)

        detections = [
            ("fire_extinguisher", mask_red),
            ("first_aid_kit", mask_green),
            ("aed_kit", mask_blue),
        ]

        debug = frame.copy()
        detected_this_frame = False

        for item_type, mask in detections:
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # pick best contour(s)
            for c in contours:
                area = cv2.contourArea(c)
                if area < self.min_area:
                    continue

                if not self._shape_ok(item_type, c):
                    continue

                detected_this_frame = True

                # publish polygon like the workshop node does
                self._publish_polygon(c, frame_id="image")

                # de-dup and count
                if not self._is_duplicate(item_type):
                    self._record_detection(item_type)

                # debug draw
                if self.show_debug:
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(debug, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(debug, item_type, (x, y - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if self.show_debug and detected_this_frame:
            now = time.time()
            if now - self._last_show >= self.debug_period:
                self._last_show = now
                cv2.imshow("InventoryDetector", debug)
                cv2.waitKey(1)

    def destroy_node(self):
        self.save_results()
        self.get_logger().info(f"Final counts: {self.counts()}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InventoryDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
