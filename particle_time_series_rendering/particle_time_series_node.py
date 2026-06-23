from __future__ import annotations

from datetime import datetime
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from particle_time_series_rendering.render_utils import (
    build_overlay_colors,
    occupancy_grid_to_bgr,
    particle_arrays,
)

try:
    from nav2_msgs.msg import ParticleCloud
except ImportError:  # pragma: no cover - optional dependency at runtime
    ParticleCloud = None


class ParticleTimeSeriesRenderingNode(Node):
    def __init__(self) -> None:
        super().__init__("particle_time_series_rendering")

        self.declare_parameter("output_mode", "publish")
        self.declare_parameter("overlay_count", 5)
        self.declare_parameter("time_interval", 1.0)
        self.declare_parameter("save_directory", "/tmp")
        self.declare_parameter("particle_topic", "/particlecloud")
        self.declare_parameter("particle_topic_type", "geometry_msgs/PoseArray")
        self.declare_parameter("output_topic", "/particle_trajectory_image")

        self.output_mode = str(self.get_parameter("output_mode").value)
        self.overlay_count = max(1, int(self.get_parameter("overlay_count").value))
        self.time_interval = max(0.0, float(self.get_parameter("time_interval").value))
        self.save_directory = Path(str(self.get_parameter("save_directory").value))
        self.particle_topic = str(self.get_parameter("particle_topic").value)
        self.particle_topic_type = str(self.get_parameter("particle_topic_type").value)
        self.output_topic = str(self.get_parameter("output_topic").value)

        if self.output_mode not in {"publish", "save"}:
            self.get_logger().warning(
                "Invalid output_mode '%s'; falling back to 'publish'." % self.output_mode
            )
            self.output_mode = "publish"

        self.bridge = CvBridge()
        self.overlay_colors = build_overlay_colors(self.overlay_count)
        self.base_map_image: np.ndarray | None = None
        self.canvas_image: np.ndarray | None = None
        self.held_image: np.ndarray | None = None
        self.overlay_index = 0
        self.last_sample_time = None
        self.waiting_for_save = False
        self.map_info = None

        self.image_publisher = self.create_publisher(Image, self.output_topic, 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid, "/map", self.map_callback, 10
        )
        self.particle_subscription = self._create_particle_subscription()
        self.save_service = self.create_service(Trigger, "~/save_image", self.handle_save_image)

        self.get_logger().info(
            "Particle renderer started in '%s' mode with overlay_count=%d time_interval=%.2fs"
            % (self.output_mode, self.overlay_count, self.time_interval)
        )

    def _create_particle_subscription(self):
        if self.particle_topic_type == "geometry_msgs/PoseArray":
            return self.create_subscription(PoseArray, self.particle_topic, self.pose_array_callback, 50)

        if self.particle_topic_type == "nav2_msgs/ParticleCloud":
            if ParticleCloud is None:
                raise RuntimeError(
                    "particle_topic_type is nav2_msgs/ParticleCloud but nav2_msgs is not installed."
                )
            return self.create_subscription(
                ParticleCloud, self.particle_topic, self.particle_cloud_callback, 50
            )

        raise RuntimeError(
            "Unsupported particle_topic_type '%s'. Use geometry_msgs/PoseArray or "
            "nav2_msgs/ParticleCloud." % self.particle_topic_type
        )

    def map_callback(self, message: OccupancyGrid) -> None:
        self.map_info = message.info
        self.base_map_image = occupancy_grid_to_bgr(
            message.data,
            message.info.width,
            message.info.height,
        )
        self.reset_canvas(clear_hold=True)
        self.get_logger().info(
            "Received map %dx%d (resolution=%.3f)."
            % (message.info.width, message.info.height, message.info.resolution)
        )

    def pose_array_callback(self, message: PoseArray) -> None:
        self.process_particle_update(message.poses)

    def particle_cloud_callback(self, message) -> None:
        self.process_particle_update([particle.pose for particle in message.particles])

    def process_particle_update(self, poses: Iterable) -> None:
        if self.base_map_image is None or self.canvas_image is None or self.map_info is None:
            self.get_logger().debug("Skipping particle update because map is not ready yet.")
            return

        if self.output_mode == "save" and self.waiting_for_save:
            return

        now = self.get_clock().now()
        if self.last_sample_time is not None:
            elapsed = (now - self.last_sample_time).nanoseconds / 1e9
            if elapsed < self.time_interval:
                return

        poses = list(poses)
        if not poses:
            return

        self.draw_particles(poses)
        self.last_sample_time = now
        self.overlay_index += 1

        if self.overlay_index >= self.overlay_count:
            finished_image = self.canvas_image.copy()
            if self.output_mode == "publish":
                self.publish_image(finished_image)
                self.reset_canvas(clear_hold=True)
            else:
                self.held_image = finished_image
                self.waiting_for_save = True
                self.get_logger().info("Overlay buffer is ready. Call ~/save_image to persist it.")

    def draw_particles(self, poses: list) -> None:
        assert self.canvas_image is not None
        assert self.map_info is not None

        color = self.overlay_colors[min(self.overlay_index, self.overlay_count - 1)]
        px, py = particle_arrays(
            poses,
            self.map_info.resolution,
            self.map_info.origin.position.x,
            self.map_info.origin.position.y,
            self.canvas_image.shape[0],
        )

        if px.size == 0:
            return

        valid = (
            (px >= 0)
            & (py >= 0)
            & (px < self.canvas_image.shape[1])
            & (py < self.canvas_image.shape[0])
        )
        if not np.any(valid):
            return

        self.canvas_image[py[valid], px[valid]] = color

    def publish_image(self, image: np.ndarray) -> None:
        image_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.image_publisher.publish(image_message)
        self.get_logger().info("Published particle trajectory image.")

    def handle_save_image(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if self.base_map_image is None:
            response.success = False
            response.message = "Cannot save image because the map has not been received yet."
            return response

        if self.output_mode == "save":
            if self.held_image is None:
                response.success = False
                response.message = "No completed overlay image is waiting to be saved."
                return response
            image_to_save = self.held_image
        else:
            image_to_save = self.canvas_image if self.canvas_image is not None else self.base_map_image
            self.get_logger().warning(
                "save_image requested while in publish mode; saving current canvas as auxiliary output."
            )

        self.save_directory.mkdir(parents=True, exist_ok=True)
        file_path = self.save_directory / self._build_output_filename()
        saved = cv2.imwrite(str(file_path), image_to_save)

        if not saved:
            response.success = False
            response.message = f"Failed to save image to {file_path}."
            return response

        if self.output_mode == "save":
            self.reset_canvas(clear_hold=True)

        response.success = True
        response.message = f"Saved image to {file_path}."
        self.get_logger().info(response.message)
        return response

    def reset_canvas(self, clear_hold: bool) -> None:
        if self.base_map_image is None:
            return

        self.canvas_image = self.base_map_image.copy()
        self.overlay_index = 0
        self.waiting_for_save = False
        if clear_hold:
            self.held_image = None

    @staticmethod
    def _build_output_filename() -> str:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"particle_history_{timestamp}.png"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ParticleTimeSeriesRenderingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
