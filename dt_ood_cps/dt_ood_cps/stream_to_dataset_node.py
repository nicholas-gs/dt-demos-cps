#!/usr/bin/env python3

import os
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge

from dt_ood_interfaces_cps.msg import DetectorInput


class StreamToDatasetNode(Node):
    """Utility node which subscribes to a topic publishing `DetectorInput`
    messages, cuts out each detected segment from the image and saves it into a
    directory, creating an image dataset.

    Each image is named as {frame number}_{segment number}. For each frame, if
    there are N number of segments, then there will be N number of images
    created.

    A random directory will be created in the `export_bin` directory to save
    the created images.

    It may be necessary to reduce the publishing rate of the `~/ood_input` to
    dropped lost frames.
    """
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.load_launch_parameters()

        self._export_dir = self.create_export_dir(self._export_bin)

        # Counter to keep track of image received
        self.img_counter = 1

        self.bridge = CvBridge()

        self.sub_ood_input = self.create_subscription(
            DetectorInput,
            "~/ood_input",
            self.cb_ood_input,
            100)

        self.get_logger().info("Initialized")

    def load_launch_parameters(self):
        self.declare_parameter("export_bin")

        self._export_bin = self.get_parameter("export_bin")\
            .get_parameter_value().string_value

    def create_export_dir(self, export_bin: str):
        """Creat a directory in the given `export_bin`. All images are
        saved in this new directory."""
        if not os.path.isdir(export_bin):
            error_msg = "f{export_bin} does not exist"
            raise ValueError(error_msg)

        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        random_dir = os.path.join(export_bin, f"dataset_{sec}_{nanosec}")
        os.makedirs(random_dir)
        self.get_logger().info(f"Results stored in {random_dir}")

        return random_dir

    def cb_ood_input(self, msg: DetectorInput):
        if self.img_counter == 1:
            self.get_logger().info("Received first image!")
        uncompressed_img = self.bridge.compressed_imgmsg_to_cv2(
            msg.frame, "bgr8")
        lines, color_ids = StreamToDatasetNode._extract_lines(msg.lines)

        for idx, line in enumerate(lines):
            cropped_img = StreamToDatasetNode._crop_segment(
                uncompressed_img, line)
            file_path = os.path.join(
                self._export_dir, f"{self.img_counter}_{idx}.png")
            cv2.imwrite(file_path, cropped_img)

        self.img_counter += 1

    @staticmethod
    def _extract_lines(lines):
        xys = []
        color_ids = []
        for line in lines:
            p1 = line.coordinates[0]
            p2 = line.coordinates[1]
            xys.append([p1.x, p1.y, p2.x, p2.y])
            color_ids.append(line.color)
        return np.array(xys, np.int), color_ids

    @staticmethod
    def _crop_segment(frame, line: np.array, thickness: int=1):
        """Crop an image using the a single bounded lines.
        """
        background = np.zeros_like(frame)
        background = cv2.line(
            background, (line[0], line[1]), (line[2], line[3]),
            color=(255,255,255), thickness=thickness)
        return cv2.bitwise_and(background, frame)


def main(args=None):
    rclpy.init(args=args)
    node = StreamToDatasetNode("stream_to_dataset_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
