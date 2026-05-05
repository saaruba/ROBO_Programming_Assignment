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
    """
    Main ROS2 node for detecting emergency equipment.

    This node runs continuously while the robot is moving.
    It subscribes to RGB image, depth image, camera information,
    and AMCL robot pose.

    The main job of this node is:
      - detect coloured emergency objects
      - estimate their map position
      - avoid duplicate counting
      - save final results into results.json
    """

    def __init__(self):
        # Create ROS2 node called inventory_node
        super().__init__("inventory_node")

        # CvBridge converts ROS Image messages into OpenCV images
        self.bridge = CvBridge()

        # RGB image topic from the robot camera
        self.color_image_topic = "/limo/depth_camera_link/image_raw"

        # Depth image topic from the depth camera
        self.depth_image_topic = "/limo/depth_camera_link/depth/image_raw"

        # Camera calibration information for RGB camera
        self.color_camera_info_topic = "/limo/depth_camera_link/camera_info"

        # Camera calibration information for depth camera
        self.depth_camera_info_topic = "/limo/depth_camera_link/depth/camera_info"

        # AMCL pose topic gives robot pose in the map frame
        self.pose_topic = "/amcl_pose"

        # Camera frame used in TF transform
        # This is the frame where camera measurements are first calculated
        self.camera_frame = "depth_link"

        # Global frame used by Nav2 and AMCL
        # All final object positions are stored in this map frame
        self.global_frame = "map"

        # These store the camera intrinsic model created from CameraInfo
        self.color_camera_model: Optional[image_geometry.PinholeCameraModel] = None
        self.depth_camera_model: Optional[image_geometry.PinholeCameraModel] = None

        # Stores the most recent depth image
        self.latest_depth_ros: Optional[Image] = None

        # Ratio used to map RGB image pixel location into depth image location
        self.color2depth_aspect: Optional[float] = None


        # Sometimes RGB-D projection or TF can fail.
        # In that case, the system falls back to the robot's AMCL pose.
        self.have_robot_pose = False
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Subscribe to RGB camera calibration data
        self.create_subscription(
            CameraInfo,
            self.color_camera_info_topic,
            self.color_camera_info_cb,
            qos.qos_profile_sensor_data,
        )

        # Subscribe to depth camera calibration data
        self.create_subscription(
            CameraInfo,
            self.depth_camera_info_topic,
            self.depth_camera_info_cb,
            qos.qos_profile_sensor_data,
        )

        # Subscribe to depth image
        self.create_subscription(
            Image,
            self.depth_image_topic,
            self.depth_image_cb,
            qos.qos_profile_sensor_data,
        )

        # Subscribe to RGB image
        self.create_subscription(
            Image,
            self.color_image_topic,
            self.color_image_cb,
            qos.qos_profile_sensor_data,
        )

        # Subscribe to AMCL pose for fallback localisation
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_cb,
            10,
        )
        # Publishes a polygon around detected object in image frame
        self.poly_pub = self.create_publisher(
            PolygonStamped,
            "/object_polygon",
            10,
        )

        # Publishes estimated object position in map frame
        self.object_location_pub = self.create_publisher(
            PoseStamped,
            "/object_location",
            10,
        )
        # TF is used to transform object pose from camera frame to map frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Stores accepted object positions.
        # Each object type has a list of map-frame anchors.
        self.detection_anchors: Dict[str, List[Tuple[float, float, float]]] = {
            "fire_extinguisher": [],
            "first_aid_kit": [],
            "aed_kit": [],
        }

        # Stores grid cells already used for detections
        # This helps reject repeated detections in same map area
        self.seen_cells = set()

        # Grid size in metres for spatial hashing
        self.grid_size = 1.0

        # If a new detection is within this distance of an old detection,
        # it is treated as the same object
        self.dedup_radius_m = 3.0

        # This prevents repeated counting while same object is continuously visible
        self.sighting_timeout_s = 2.0

        # True means this object type is currently visible and locked
        self.active_sighting = {
            "fire_extinguisher": False,
            "first_aid_kit": False,
            "aed_kit": False,
        }

        # Stores last time each object type was seen
        self.last_seen_time = {
            "fire_extinguisher": 0.0,
            "first_aid_kit": 0.0,
            "aed_kit": 0.0,
        }

        self.out_path = Path(
            "/workspaces/cmp9767-ws-main/src/emergency_inventory/emergency_inventory/results.json"
        )

        # HSV is used instead of RGB because it is better for colour detection.
        # Red uses two ranges because red wraps around the HSV hue scale.
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

        # Minimum contour area to reject tiny noise
        self.min_area = 100

        # Fire extinguisher should usually look taller than wide
        self.extinguisher_min_aspect = 0.7

        # First aid kit and AED are box-like, so aspect ratio is kept flexible
        self.box_min_aspect = 0.25
        self.box_max_aspect = 4.0

        # Extent = contour area / bounding box area
        # Low extent means blob is too scattered or noisy
        self.min_extent = 0.08

        # Ignore invalid depth that is too close or too far
        self.min_depth_m = 0.10
        self.max_depth_m = 10.0

        self.show_debug = True
        self.debug_period = 0.25
        self._last_show = 0.0

        self.get_logger().info("InventoryDetector started.")
        self.get_logger().info(
            "Using RGB-D projection, AMCL fallback, and sighting-lock deduplication."
        )

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        """
        Receives robot pose from AMCL.

        This is used as a fallback when object projection into map frame fails.
        """
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.have_robot_pose = True

    def color_camera_info_cb(self, msg: CameraInfo):
        """
        Receives RGB camera calibration data.

        CameraInfo is needed to convert image pixels into 3D rays.
        """
        if self.color_camera_model is None:
            self.color_camera_model = image_geometry.PinholeCameraModel()
            self.color_camera_model.fromCameraInfo(msg)
            self.get_logger().info("Colour camera model received.")
            self.calculate_color2depth_aspect()

    def depth_camera_info_cb(self, msg: CameraInfo):
        """
        Receives depth camera calibration data.
        """
        if self.depth_camera_model is None:
            self.depth_camera_model = image_geometry.PinholeCameraModel()
            self.depth_camera_model.fromCameraInfo(msg)
            self.get_logger().info("Depth camera model received.")
            self.calculate_color2depth_aspect()

    def depth_image_cb(self, msg: Image):
        """
        Stores latest depth image.

        The RGB callback later uses this depth image to estimate object distance.
        """
        self.latest_depth_ros = msg


    def calculate_color2depth_aspect(self):
        """
        Calculates ratio between RGB image and depth image.

        This helps find the corresponding depth pixel for a colour image pixel.
        """
        if (
            self.color2depth_aspect is None
            and self.color_camera_model is not None
            and self.depth_camera_model is not None
        ):
            color_angle_per_pixel = (
                math.atan2(
                    self.color_camera_model.width,
                    2.0 * self.color_camera_model.fx(),
                )
                / self.color_camera_model.width
            )

            depth_angle_per_pixel = (
                math.atan2(
                    self.depth_camera_model.width,
                    2.0 * self.depth_camera_model.fx(),
                )
                / self.depth_camera_model.width
            )

            self.color2depth_aspect = color_angle_per_pixel / depth_angle_per_pixel

            self.get_logger().info(
                f"Colour-to-depth aspect calculated: {self.color2depth_aspect:.3f}"
            )


    def mask_from_ranges(self, hsv: np.ndarray, ranges) -> np.ndarray:
        """
        Creates a binary mask from one or more HSV ranges.

        White pixels = object colour detected
        Black pixels = background
        """
        mask = None

        for lo, hi in ranges:
            current_mask = cv2.inRange(
                hsv,
                np.array(lo, dtype=np.uint8),
                np.array(hi, dtype=np.uint8),
            )

            # If multiple ranges exist, combine them together
            mask = current_mask if mask is None else cv2.bitwise_or(mask, current_mask)

        return mask

    def clean_mask(self, mask: np.ndarray) -> np.ndarray:
        """
        Cleans binary mask using morphology.

        MORPH_OPEN removes small noise.
        MORPH_CLOSE fills small holes inside detected blobs.
        """
        kernel = np.ones((5, 5), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        return mask

    def shape_ok(self, item_type: str, contour) -> bool:
        """
        Checks whether a detected contour has a valid shape.

        This reduces false detections caused by random coloured pixels.
        """
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

        # Fire extinguisher should be relatively vertical
        if item_type == "fire_extinguisher":
            return aspect >= self.extinguisher_min_aspect

        # Box objects can have wider aspect range
        return self.box_min_aspect <= aspect <= self.box_max_aspect

    def contour_centroid(self, contour) -> Optional[Tuple[float, float]]:
        """
        Calculates centre point of contour.

        Returns:
            (row, col) = (y pixel, x pixel)
        """
        moments = cv2.moments(contour)

        if moments["m00"] == 0:
            return None

        cx = moments["m10"] / moments["m00"]
        cy = moments["m01"] / moments["m00"]

        return cy, cx


    def get_depth_at_pixel(
        self,
        depth_image: np.ndarray,
        depth_row: int,
        depth_col: int,
    ) -> Optional[float]:
        """
        Gets stable depth value around object centre.

        Instead of taking only one pixel, it takes a small patch around
        the centre and returns median valid depth. This is more stable.
        """
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
        """
        Converts 2D image pixel into 3D camera-frame pose.

        Steps:
          1. Find matching depth pixel
          2. Get depth value
          3. Project RGB pixel into 3D ray
          4. Multiply ray by depth
        """
        if self.color_camera_model is None or self.color2depth_aspect is None:
            return None

        color_shape = np.array(color_image.shape[:2], dtype=np.float32)
        depth_shape = np.array(depth_image.shape[:2], dtype=np.float32)
        image_coords_np = np.array(image_coords, dtype=np.float32)

        # Convert colour image coordinate to depth image coordinate
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

        # Project pixel into 3D ray using camera intrinsic model
        ray = np.array(
            self.color_camera_model.projectPixelTo3dRay((float(col), float(row))),
            dtype=np.float32,
        )

        if abs(ray[2]) < 1e-6:
            return None

        # Normalise ray so z = 1
        ray = ray / ray[2]

        # Scale ray using actual depth value
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
        """
        Transforms object pose from camera frame into map frame using TF.

        If TF is not available, return None and AMCL fallback will be used.
        """
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
        """
        Converts map position into grid cell key.

        This is used for fast duplicate rejection.
        """
        cx = int(math.floor(x / self.grid_size))
        cy = int(math.floor(y / self.grid_size))

        return cx, cy, item_type

    def is_duplicate(self, item_type: str, x: float, y: float) -> bool:
        """
        Checks whether detection is duplicate.

        Two checks are used:
          1. Same grid cell
          2. Too close to existing anchor point
        """
        key = self.cell_key(item_type, x, y)

        if key in self.seen_cells:
            return True

        for ax, ay, _az in self.detection_anchors[item_type]:
            distance = math.hypot(ax - x, ay - y)

            if distance < self.dedup_radius_m:
                return True

        return False

    def record_detection(self, item_type: str, x: float, y: float, z: float):
        """
        Records a valid new detection.

        The detection is saved into:
          - seen grid cells
          - detection anchors
          - results.json file
        """
        key = self.cell_key(item_type, x, y)

        self.seen_cells.add(key)
        self.detection_anchors[item_type].append((x, y, z))

        self.get_logger().info(
            f"Detected {item_type} at map position ({x:.2f}, {y:.2f}, {z:.2f})"
        )

        self.save_results()

    def counts(self) -> Dict[str, int]:
        """
        Counts number of accepted detections for each object type.
        """
        return {item: len(points) for item, points in self.detection_anchors.items()}

    def save_results(self):
        """
        Saves inventory results into JSON file.

        This file is final output of the robot inspection mission.
        """
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
        """
        Publishes bounding box polygon around detected object.

        Useful for RViz/debugging.
        """
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
        """
        Publishes estimated object pose in map frame.
        """
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.global_frame
        msg.pose = map_pose

        self.object_location_pub.publish(msg)


    def color_image_cb(self, msg: Image):
        """
        Main detection pipeline.

        This function runs every time a new RGB image arrives.
        """
        # Wait until all camera models and depth image are ready
        if (
            self.color_camera_model is None
            or self.depth_camera_model is None
            or self.color2depth_aspect is None
            or self.latest_depth_ros is None
        ):
            return

        try:
            # Convert ROS image messages to OpenCV images
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            depth_image = self.bridge.imgmsg_to_cv2(
                self.latest_depth_ros,
                desired_encoding="32FC1",
            )

        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            return

        # Convert BGR image into HSV colour space
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # Create clean masks for each object type
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

        # Copy image for drawing debug boxes
        debug = color_image.copy()

        # Tracks which object types are visible in current frame
        seen_types_this_frame = set()

        # Process each object type separately
        for item_type, mask in masks.items():
            # Find contours in binary mask
            contours, _ = cv2.findContours(
                mask,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )

            if not contours:
                continue

            # Process biggest contours first
            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            for contour in contours:
                # Reject contour if shape is not valid
                if not self.shape_ok(item_type, contour):
                    continue

                # Calculate centre of detected object
                centroid = self.contour_centroid(contour)

                if centroid is None:
                    continue

                # Convert 2D image point to 3D camera pose
                camera_pose = self.image_to_camera_pose(
                    centroid,
                    color_image,
                    depth_image,
                )

                map_pose = None

                # Try to convert camera-frame pose into map-frame pose
                if camera_pose is not None:
                    map_pose = self.camera_pose_to_map_pose(camera_pose)

                # If TF projection works, use object map position
                if map_pose is not None:
                    obj_x = map_pose.position.x
                    obj_y = map_pose.position.y
                    obj_z = map_pose.position.z
                    self.publish_object_location(map_pose)

                # If TF/depth fails, use robot AMCL pose as fallback
                elif self.have_robot_pose:
                    obj_x = self.robot_x
                    obj_y = self.robot_y
                    obj_z = 0.0

                # If no map position is available, skip this detection
                else:
                    continue

                # Mark this object type as seen in current frame
                seen_types_this_frame.add(item_type)
                self.last_seen_time[item_type] = time.time()

                # Publish polygon for visual debug
                self.publish_polygon(contour, frame_id=msg.header.frame_id)

                # If this object type is not currently locked,
                # check if it is a new object and record it.
                if not self.active_sighting[item_type]:
                    if not self.is_duplicate(item_type, obj_x, obj_y):
                        self.record_detection(item_type, obj_x, obj_y, obj_z)

                    # Lock this object type while it remains visible
                    self.active_sighting[item_type] = True

                # Draw debug box and label on image
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

                # Only use best valid contour for each object type per frame
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
        """
        Called when node shuts down.

        Saves final results and closes OpenCV windows.
        """
        self.save_results()
        self.get_logger().info(f"Final counts: {self.counts()}")
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """
    Main entry point for ROS2 node.
    """
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