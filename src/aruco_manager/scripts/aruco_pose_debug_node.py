#!/usr/bin/env python3
"""
aruco_pose_debug_node.py

Subscribes to a camera image and a DetectedTagArray, then publishes an
annotated debug image showing for each active tag:
  - Projected tag centre (crosshair + circle)
  - Tag ID, XYZ pose and distance in cm
  - Confidence bar (red → yellow → green)

Published topic: ~/debug_image  (sensor_msgs/Image, BGR8)

Parameters:
  camera_topic   – raw camera image topic  (default: /camera/image_raw)
  tags_topic     – DetectedTagArray topic  (default: /findeeznuts/detected_tags)
  show_window    – open local cv2 window   (default: false, use on desktop only)
  camera_matrix  – 9-element flat array matching aruco_picker.yaml
"""

import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from aruco_interfaces.msg import ClusterPickability, DetectedTagArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

# ── Helpers ───────────────────────────────────────────────────────────────────

FONT       = cv2.FONT_HERSHEY_SIMPLEX
FONT_SMALL = 0.45
FONT_MED   = 0.55
LINE       = cv2.LINE_AA
WHITE      = (255, 255, 255)
DARK       = (30,  30,  30)
STATUS_BG  = (18,  18,  18)
STATUS_BORDER = (90, 90, 90)


def _conf_color(conf: float):
    """BGR colour: red (0) → yellow (0.5) → green (1)."""
    conf = max(0.0, min(1.0, conf))
    if conf < 0.5:
        r = 255
        g = int(255 * conf * 2)
    else:
        r = int(255 * (1.0 - conf) * 2)
        g = 255
    return (0, g, r)


def _shadow_text(img, text, org, font, scale, color, thickness=1):
    """Draw text with a dark outline for readability on any background."""
    cv2.putText(img, text, org, font, scale, DARK,  thickness + 1, LINE)
    cv2.putText(img, text, org, font, scale, color, thickness,     LINE)


# ═════════════════════════════════════════════════════════════════════════════
class ArucoPosgDebugNode(Node):

    def __init__(self):
        super().__init__("aruco_pose_debug")

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("tags_topic",   "/findeeznuts/detected_tags")
        self.declare_parameter("pickability_topic", "/cluster_pickability")
        self.declare_parameter("show_window",  False)
        self.declare_parameter("arm_x_origin", -0.075)
        self.declare_parameter("arm_y_origin", 0.0)
        self.declare_parameter("arm_spacing", 0.05)
        default_picker_yaml = os.path.join(
            get_package_share_directory("aruco_manager"),
            "config",
            "aruco_picker.yaml",
        )
        self.declare_parameter("camera_config_file", default_picker_yaml)
        self.declare_parameter("camera_matrix",
            [600.0, 0.0, 320.0,
               0.0, 600.0, 240.0,
               0.0,   0.0,   1.0])
        self.declare_parameter("dist_coeffs", [0.0, 0.0, 0.0, 0.0, 0.0])

        cam_topic  = self.get_parameter("camera_topic").value
        tags_topic = self.get_parameter("tags_topic").value
        pick_topic = self.get_parameter("pickability_topic").value
        self._show = self.get_parameter("show_window").value
        self._arm_x_origin = float(self.get_parameter("arm_x_origin").value)
        self._arm_y_origin = float(self.get_parameter("arm_y_origin").value)
        self._arm_spacing = float(self.get_parameter("arm_spacing").value)
        self._load_camera_config()
        self._bridge      = CvBridge()
        self._latest_tags = None   # type: DetectedTagArray | None
        self._latest_pick = None   # type: ClusterPickability | None

        # ── Subscriptions ─────────────────────────────────────────────────────
        self._tag_sub = self.create_subscription(
            DetectedTagArray, tags_topic, self._tags_cb, be_qos)

        self._pick_sub = self.create_subscription(
            ClusterPickability, pick_topic, self._pick_cb, be_qos)

        self._img_sub = self.create_subscription(
            Image, cam_topic, self._image_cb, be_qos)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(Image, "~/debug_image", be_qos)

        self.get_logger().info(
            f"aruco_pose_debug ready | cam={cam_topic} | tags={tags_topic} "
            f"| pickability={pick_topic} | publish -> ~/debug_image")

    def _load_camera_config(self):
        cm = self.get_parameter("camera_matrix").value
        dc = self.get_parameter("dist_coeffs").value
        cfg_path = self.get_parameter("camera_config_file").value

        if cfg_path and os.path.isfile(cfg_path):
            try:
                with open(cfg_path, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                params = data.get("/**", {}).get("ros__parameters", {})
                if "camera_matrix" in params and len(params["camera_matrix"]) == 9:
                    cm = params["camera_matrix"]
                if "dist_coeffs" in params and len(params["dist_coeffs"]) >= 5:
                    dc = params["dist_coeffs"]
                self.get_logger().info(f"Loaded camera calibration from {cfg_path}")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Failed to read camera config '{cfg_path}': {exc}")

        self._fx = float(cm[0])
        self._fy = float(cm[4])
        self._cx = float(cm[2])
        self._cy = float(cm[5])
        self._cam_mat = np.array(
            [[self._fx, 0.0, self._cx],
             [0.0, self._fy, self._cy],
             [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        self._dist_coeffs = np.array(list(dc)[:5], dtype=np.float64).reshape(-1, 1)

    def _project_point(self, x: float, y: float, z: float):
        if z <= 0.001:
            return None
        obj = np.array([[x, y, z]], dtype=np.float64)
        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.zeros((3, 1), dtype=np.float64)
        img_pts, _ = cv2.projectPoints(obj, rvec, tvec, self._cam_mat, self._dist_coeffs)
        return int(img_pts[0, 0, 0]), int(img_pts[0, 0, 1])


    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _tags_cb(self, msg: DetectedTagArray):
        self._latest_tags = msg

    def _pick_cb(self, msg: ClusterPickability):
        self._latest_pick = msg

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"cv_bridge error: {exc}", throttle_duration_sec=2.0)
            return

        out = self._draw_overlays(frame)

        out_msg          = self._bridge.cv2_to_imgmsg(out, encoding="bgr8")
        out_msg.header   = msg.header
        self._pub.publish(out_msg)

        if self._show:
            cv2.imshow("ArUco Pose Debug", out)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                cv2.destroyAllWindows()
                rclpy.shutdown()

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _draw_overlays(self, frame: np.ndarray) -> np.ndarray:
        feed = frame.copy()
        h, w = feed.shape[:2]
        status_h = 128
        center_u = w // 2
        center_v = h // 2

        cv2.circle(feed, (center_u, center_v), 5, (255, 255, 255), -1, LINE)
        cv2.circle(feed, (center_u, center_v), 3, (0, 0, 255), -1, LINE)

        tags = self._latest_tags
        if tags is not None and tags.tags:
            for tag in tags.tags:
                p    = tag.tag_pose.position
                x, y, z = float(p.x), float(p.y), float(p.z)
                conf = float(tag.confidence)
                color = _conf_color(conf)
                track_id = self._find_track_id_for_tag_pose(x, y, z)

                uv = self._project_point(x, y, z)
                if uv is None:
                    continue
                u, v = uv

                # Clamp to image bounds for drawing (can still be off-screen)
                u_clamp = max(0, min(w - 1, u))
                v_clamp = max(0, min(h - 1, v))

                # ── Crosshair + circle ─────────────────────────────────────────────
                R = 16
                cv2.circle(feed, (u, v), R, color, 2, LINE)
                cv2.line(feed, (u - R, v), (u + R, v), color, 1, LINE)
                cv2.line(feed, (u, v - R), (u, v + R), color, 1, LINE)

                # Keep only tracker id label on tag overlays.
                label = f"T:{track_id}" if track_id >= 0 else "T:-"
                text_x = max(6, min(w - 50, u_clamp - 20))
                text_y = max(18, v_clamp - R - 8)
                _shadow_text(feed, label, (text_x, text_y), FONT, FONT_SMALL, color)

                # Keep confidence bar visualization.
                bar_w  = 64
                bar_h  = 8
                bx     = max(0, min(w - bar_w, u_clamp - bar_w // 2))
                by     = max(0, min(h - bar_h - 1, v_clamp + R + 6))
                cv2.rectangle(feed, (bx, by), (bx + bar_w, by + bar_h), (50, 50, 50), -1)
                filled = max(1, int(bar_w * conf))
                cv2.rectangle(feed, (bx, by), (bx + filled, by + bar_h), color, -1)
                cv2.rectangle(feed, (bx, by), (bx + bar_w, by + bar_h), (150, 150, 150), 1)

        self._draw_assignment_overlays(feed)

        out = np.full((h + status_h, w, 3), STATUS_BG, dtype=np.uint8)
        out[status_h:status_h + h, :w] = feed
        cv2.line(out, (0, status_h - 1), (w - 1, status_h - 1), STATUS_BORDER, 1, LINE)
        self._draw_status_panel(out, status_h)

        return out

    def _find_track_id_for_tag_pose(self, x: float, y: float, z: float) -> int:
        pick = self._latest_pick
        if pick is None:
            return -1
        best_track_id = -1
        best_dist_sq = float("inf")
        for aa in pick.arms:
            if not aa.assigned:
                continue
            p = aa.tag_pose.position
            dx = float(p.x) - x
            dy = float(p.y) - y
            dz = float(p.z) - z
            dist_sq = dx * dx + dy * dy + dz * dz
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_track_id = int(aa.track_id)
        return best_track_id

    def _draw_assignment_overlays(self, feed: np.ndarray):
        pick = self._latest_pick
        if pick is None:
            return

        for aa in pick.arms:
            if not aa.assigned:
                continue

            p = aa.tag_pose.position
            x, y, z = float(p.x), float(p.y), float(p.z)
            if z <= 0.001:
                continue

            start_uv = self._project_point(x, y, z)
            if start_uv is None:
                continue
            u0, v0 = start_uv

            # Goal point from configured arm geometry (tag_config.yaml params).
            arm_idx = int(aa.arm_index)
            gx = self._arm_x_origin + arm_idx * self._arm_spacing
            gy = self._arm_y_origin
            goal_uv = self._project_point(gx, gy, z)
            if goal_uv is None:
                continue
            u1, v1 = goal_uv

            color = (255, 220, 0)
            cv2.circle(feed, (u0, v0), 5, (0, 200, 255), -1, LINE)
            square_half = 6
            cv2.rectangle(
                feed,
                (u1 - square_half, v1 - square_half),
                (u1 + square_half, v1 + square_half),
                (80, 255, 80),
                2,
                LINE,
            )
            cv2.line(feed, (u0, v0), (u1, v1), color, 2, LINE)

    def _draw_status_panel(self, out: np.ndarray, status_h: int):
        pick = self._latest_pick
        tags = self._latest_tags
        tag_count = len(tags.tags) if tags is not None else 0

        panel_x, panel_y = 8, 8
        panel_w = out.shape[1] - 16
        panel_h = status_h - 16
        cv2.rectangle(out, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), STATUS_BG, -1)
        cv2.rectangle(out, (panel_x, panel_y), (panel_x + panel_w, panel_y + panel_h), STATUS_BORDER, 1)

        if pick is None:
            _shadow_text(
                out,
                f"Pickability: waiting...  visible_tags:{tag_count}",
                (panel_x + 10, panel_y + 20),
                FONT,
                FONT_SMALL,
                WHITE,
            )
            _shadow_text(
                out,
                "Top banner=status, lower area=camera feed with tracking IDs, confidence bars, and arm geometry.",
                (panel_x + 10, panel_y + 42),
                FONT,
                FONT_SMALL - 0.03,
                (190, 190, 190),
            )
            return

        corr_x = float(pick.correction.x)
        corr_y = float(pick.correction.y)
        corr_theta = float(pick.correction.z)
        corr_theta_deg = math.degrees(corr_theta)
        state_color = (30, 220, 30) if pick.is_pickable else (0, 120, 255)
        sticky_color = (0, 90, 255) if pick.cluster_lost else ((30, 220, 30) if pick.sticky_active else (180, 180, 180))

        _shadow_text(
            out,
            (
                f"Pickable:{pick.is_pickable}  cluster:{int(pick.cluster_id)}  "
                f"assigned:{int(pick.assigned_count)}/{int(pick.total_tags)}  visible:{tag_count}"
            ),
            (panel_x + 10, panel_y + 20),
            FONT,
            FONT_SMALL,
            state_color,
        )
        _shadow_text(
            out,
            (
                f"Sticky:{pick.sticky_active}  cluster_lost:{pick.cluster_lost}  "
                f"lost_frames:{int(pick.lost_tracking_frames)}"
            ),
            (panel_x + 10, panel_y + 40),
            FONT,
            FONT_SMALL,
            sticky_color,
        )
        _shadow_text(
            out,
            f"Error |corr|: {pick.correction_magnitude:.3f} m ({pick.correction_magnitude*100.0:.1f} cm)",
            (panel_x + 10, panel_y + 60),
            FONT,
            FONT_SMALL,
            WHITE,
        )
        _shadow_text(
            out,
            f"Correction x:{corr_x:+.3f} m  y:{corr_y:+.3f} m  theta:{corr_theta:+.3f} rad ({corr_theta_deg:+.1f} deg)",
            (panel_x + 10, panel_y + 80),
            FONT,
            FONT_SMALL,
            WHITE,
        )

        arm_chunks = []
        for aa in pick.arms:
            if aa.assigned:
                arm_chunks.append(f"A{int(aa.arm_index)}->ID{int(aa.tag_id)} T{int(aa.track_id)}")
            else:
                arm_chunks.append(f"A{int(aa.arm_index)}->-")
        _shadow_text(
            out,
            "  ".join(arm_chunks),
            (panel_x + 10, panel_y + 100),
            FONT,
            FONT_SMALL,
            (220, 220, 220),
        )


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = ArucoPosgDebugNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
