import json
import math
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import image_geometry
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    Point,
    Point32,
    PolygonStamped,
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
)
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class InventoryDetector(Node):
    def __init__(self):
        super().__init__("inventory_node")

        self.bridge = CvBridge()

        self.color_image_topic = "/limo/depth_camera_link/image_raw"
        self.depth_image_topic = "/limo/depth_camera_link/depth/image_raw"
        self.color_camera_info_topic = "/limo/depth_camera_link/camera_info"
        self.depth_camera_info_topic = "/limo/depth_camera_link/depth/camera_info"
        self.pose_topic = "/amcl_pose"

        self.camera_frame = "depth_link"
        self.global_frame = "map"

        self.color_camera_model: Optional[image_geometry.PinholeCameraModel] = None
        self.depth_camera_model: Optional[image_geometry.PinholeCameraModel] = None
        self.latest_depth_ros: Optional[Image] = None
        self.color2depth_aspect: Optional[float] = None

        self.have_robot_pose = False
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.create_subscription(
            CameraInfo,
            self.color_camera_info_topic,
            self.color_camera_info_cb,
            qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.depth_camera_info_topic,
            self.depth_camera_info_cb,
            qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_image_topic,
            self.depth_image_cb,
            qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.color_image_topic,
            self.color_image_cb,
            qos.qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_cb,
            10,
        )

        self.poly_pub = self.create_publisher(PolygonStamped, "/object_polygon", 10)
        self.object_location_pub = self.create_publisher(PoseStamped, "/object_location", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.detection_anchors: Dict[str, List[Tuple[float, float, float]]] = {
            "fire_extinguisher": [],
            "first_aid_kit": [],
            "aed_kit": [],
        }

        self.seen_cells = set()

        self.grid_size = 1.0
        self.dedup_radius_m = 3.0

        self.sighting_timeout_s = 2.0
        self.active_sighting = {
            "fire_extinguisher": False,
            "first_aid_kit": False,
            "aed_kit": False,
        }
        self.last_seen_time = {
            "fire_extinguisher": 0.0,
            "first_aid_kit": 0.0,
            "aed_kit": 0.0,
        }

        self.out_path = Path(
            "/workspaces/cmp9767-ws-main/src/emergency_inventory/emergency_inventory/results.json"
        )

        self.hsv_ranges = {
            "fire_extinguisher": [
                ((0, 80, 40), (12, 255, 255)),
                ((165, 80, 40), (180, 255, 255)),
            ],
            "first_aid_kit": [
                ((35, 40, 40), (90, 255, 255)),
            ],
            "aed_kit": [
                ((85, 40, 40), (140, 255, 255)),
            ],
        }

        self.min_area = 100
        self.extinguisher_min_aspect = 0.7
        self.box_min_aspect = 0.25
        self.box_max_aspect = 4.0
        self.min_extent = 0.08

        self.min_depth_m = 0.10
        self.max_depth_m = 10.0

        self.show_debug = True
        self.debug_period = 0.25
        self._last_show = 0.0

        self.get_logger().info("InventoryDetector started.")
        self.get_logger().info("Using RGB-D projection, AMCL fallback, and sighting-lock deduplication.")

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.have_robot_pose = True

    def color_camera_info_cb(self, msg: CameraInfo):
        if self.color_camera_model is None:
            self.color_camera_model = image_geometry.PinholeCameraModel()
            self.color_camera_model.fromCameraInfo(msg)
            self.get_logger().info("Colour camera model received.")
            self.calculate_color2depth_aspect()

    def depth_camera_info_cb(self, msg: CameraInfo):
        if self.depth_camera_model is None:
            self.depth_camera_model = image_geometry.PinholeCameraModel()
            self.depth_camera_model.fromCameraInfo(msg)
            self.get_logger().info("Depth camera model received.")
            self.calculate_color2depth_aspect()

    def depth_image_cb(self, msg: Image):
        self.latest_depth_ros = msg

    def calculate_color2depth_aspect(self):
        if (
            self.color2depth_aspect is None
            and self.color_camera_model is not None
            and self.depth_camera_model is not None
        ):
            color_angle_per_pixel = (
                math.atan2(self.color_camera_model.width, 2.0 * self.color_camera_model.fx())
                / self.color_camera_model.width
            )
            depth_angle_per_pixel = (
                math.atan2(self.depth_camera_model.width, 2.0 * self.depth_camera_model.fx())
                / self.depth_camera_model.width
            )
            self.color2depth_aspect = color_angle_per_pixel / depth_angle_per_pixel
            self.get_logger().info(
                f"Colour-to-depth aspect calculated: {self.color2depth_aspect:.3f}"
            )

    def mask_from_ranges(self, hsv: np.ndarray, ranges) -> np.ndarray:
        mask = None
        for lo, hi in ranges:
            current_mask = cv2.inRange(
                hsv,
                np.array(lo, dtype=np.uint8),
                np.array(hi, dtype=np.uint8),
            )
            mask = current_mask if mask is None else cv2.bitwise_or(mask, current_mask)
        return mask

    def clean_mask(self, mask: np.ndarray) -> np.ndarray:
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        return mask

    def shape_ok(self, item_type: str, contour) -> bool:
        x, y, w, h = cv2.boundingRect(contour)

        if w <= 0 or h <= 0:
            return False

        area = cv2.contourArea(contour)

        if area < self.min_area:
            return False

        rect_area = float(w * h)
        extent = area / rect_area
        aspect = float(h) / float(w)

        if extent < self.min_extent:
            return False

        if item_type == "fire_extinguisher":
            return aspect >= self.extinguisher_min_aspect

        return self.box_min_aspect <= aspect <= self.box_max_aspect

    def contour_centroid(self, contour) -> Optional[Tuple[float, float]]:
        moments = cv2.moments(contour)

        if moments["m00"] == 0:
            return None

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]

        return cy, cx

    def get_depth_at_pixel(self, depth_image: np.ndarray, depth_row: int, depth_col: int) -> Optional[float]:
        height, width = depth_image.shape[:2]

        if depth_row < 0 or depth_row >= height or depth_col < 0 or depth_col >= width:
            return None

        r1 = max(0, depth_row - 3)
        r2 = min(height, depth_row + 4)
        c1 = max(0, depth_col - 3)
        c2 = min(width, depth_col + 4)

        patch = depth_image[r1:r2, c1:c2].astype(np.float32)
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > self.min_depth_m) & (valid < self.max_depth_m)]

        if valid.size == 0:
            return None

        return float(np.median(valid))

    def image_to_camera_pose(
        self,
        image_coords: Tuple[float, float],
        color_image: np.ndarray,
        depth_image: np.ndarray,
    ) -> Optional[Pose]:
        if self.color_camera_model is None or self.color2depth_aspect is None:
            return None

        color_shape = np.array(color_image.shape[:2], dtype=np.float32)
        depth_shape = np.array(depth_image.shape[:2], dtype=np.float32)
        image_coords_np = np.array(image_coords, dtype=np.float32)

        depth_coords = (
            depth_shape / 2.0
            + (image_coords_np - color_shape / 2.0) * self.color2depth_aspect
        )

        depth_row = int(depth_coords[0])
        depth_col = int(depth_coords[1])

        depth_value = self.get_depth_at_pixel(depth_image, depth_row, depth_col)

        if depth_value is None:
            return None

        row, col = image_coords
        ray = np.array(
            self.color_camera_model.projectPixelTo3dRay((float(col), float(row))),
            dtype=np.float32,
        )

        if abs(ray[2]) < 1e-6:
            return None

        ray = ray / ray[2]
        camera_coords = ray * depth_value

        return Pose(
            position=Point(
                x=float(camera_coords[0]),
                y=float(camera_coords[1]),
                z=float(camera_coords[2]),
            ),
            orientation=Quaternion(w=1.0),
        )

    def camera_pose_to_map_pose(self, camera_pose: Pose) -> Optional[Pose]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.camera_frame,
                rclpy.time.Time(),
            )
            return do_transform_pose(camera_pose, transform)

        except TransformException:
            return None

    def cell_key(self, item_type: str, x: float, y: float) -> Tuple[int, int, str]:
        cx = int(math.floor(x / self.grid_size))
        cy = int(math.floor(y / self.grid_size))
        return cx, cy, item_type

    def is_duplicate(self, item_type: str, x: float, y: float) -> bool:
        key = self.cell_key(item_type, x, y)

        if key in self.seen_cells:
            return True

        for ax, ay, _az in self.detection_anchors[item_type]:
            distance = math.hypot(ax - x, ay - y)
            if distance < self.dedup_radius_m:
                return True

        return False

    def record_detection(self, item_type: str, x: float, y: float, z: float):
        key = self.cell_key(item_type, x, y)
        self.seen_cells.add(key)
        self.detection_anchors[item_type].append((x, y, z))

        self.get_logger().info(
            f"Detected {item_type} at map position ({x:.2f}, {y:.2f}, {z:.2f})"
        )

        self.save_results()

    def counts(self) -> Dict[str, int]:
        return {item: len(points) for item, points in self.detection_anchors.items()}

    def save_results(self):
        data = {
            "timestamp_unix": time.time(),
            "global_frame": self.global_frame,
            "camera_frame": self.camera_frame,
            "method": "RGB-D projection with AMCL fallback and sighting-lock deduplication",
            "grid_size_m": self.grid_size,
            "dedup_radius_m": self.dedup_radius_m,
            "sighting_timeout_s": self.sighting_timeout_s,
            "counts": self.counts(),
            "anchors": {
                item: [{"x": x, "y": y, "z": z} for x, y, z in anchors]
                for item, anchors in self.detection_anchors.items()
            },
        }

        try:
            self.out_path.write_text(json.dumps(data, indent=2))
        except Exception as e:
            self.get_logger().warn(f"Could not write results.json: {e}")

    def publish_polygon(self, contour, frame_id: str):
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

    def publish_object_location(self, map_pose: Pose):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.pose = map_pose
        self.object_location_pub.publish(msg)

    def color_image_cb(self, msg: Image):
        if (
            self.color_camera_model is None
            or self.depth_camera_model is None
            or self.color2depth_aspect is None
            or self.latest_depth_ros is None
        ):
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(
                self.latest_depth_ros,
                desired_encoding="32FC1",
            )
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        masks = {
            "fire_extinguisher": self.clean_mask(
                self.mask_from_ranges(hsv, self.hsv_ranges["fire_extinguisher"])
            ),
            "first_aid_kit": self.clean_mask(
                self.mask_from_ranges(hsv, self.hsv_ranges["first_aid_kit"])
            ),
            "aed_kit": self.clean_mask(
                self.mask_from_ranges(hsv, self.hsv_ranges["aed_kit"])
            ),
        }

        debug = color_image.copy()
        seen_types_this_frame = set()

        for item_type, mask in masks.items():
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            if not contours:
                continue

            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            for contour in contours:
                if not self.shape_ok(item_type, contour):
                    continue

                centroid = self.contour_centroid(contour)

                if centroid is None:
                    continue

                camera_pose = self.image_to_camera_pose(
                    centroid,
                    color_image,
                    depth_image,
                )

                map_pose = None

                if camera_pose is not None:
                    map_pose = self.camera_pose_to_map_pose(camera_pose)

                if map_pose is not None:
                    obj_x = map_pose.position.x
                    obj_y = map_pose.position.y
                    obj_z = map_pose.position.z
                    self.publish_object_location(map_pose)

                elif self.have_robot_pose:
                    obj_x = self.robot_x
                    obj_y = self.robot_y
                    obj_z = 0.0

                else:
                    continue

                seen_types_this_frame.add(item_type)
                self.last_seen_time[item_type] = time.time()

                self.publish_polygon(contour, frame_id=msg.header.frame_id)

                if not self.active_sighting[item_type]:
                    if not self.is_duplicate(item_type, obj_x, obj_y):
                        self.record_detection(item_type, obj_x, obj_y, obj_z)

                    self.active_sighting[item_type] = True

                if self.show_debug:
                    x, y, w, h = cv2.boundingRect(contour)

                    cv2.rectangle(debug, (x, y), (x + w, y + h), (255, 255, 255), 2)
                    cv2.putText(
                        debug,
                        item_type,
                        (x, max(15, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (255, 255, 255),
                        2,
                    )

                    cy, cx = centroid
                    cv2.circle(debug, (int(cx), int(cy)), 5, (255, 255, 255), -1)

                break

        now_time = time.time()

        for item_type in self.active_sighting:
            if item_type not in seen_types_this_frame:
                if now_time - self.last_seen_time[item_type] > self.sighting_timeout_s:
                    self.active_sighting[item_type] = False

        if self.show_debug:
            now = time.time()

            if now - self._last_show >= self.debug_period:
                self._last_show = now
                cv2.imshow("InventoryDetector RGB-D", debug)
                cv2.waitKey(1)

    def destroy_node(self):
        self.save_results()
        self.get_logger().info(f"Final counts: {self.counts()}")
        cv2.destroyAllWindows()
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