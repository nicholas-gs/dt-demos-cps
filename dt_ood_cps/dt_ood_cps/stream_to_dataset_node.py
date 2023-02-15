#!/usr/bin/env python3

import os
import cv2
import rclpy
import numpy as np

from typing import Tuple
from rclpy.node import Node
from cv_bridge import CvBridge

from dt_ood_cps.cropper import (
    BinCropper,
    FitCropper,
    TrimCropper,
    PassthroughCropper
)
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

    CROPPERS = {
        "bin" : BinCropper,
        "fit" : FitCropper,
        "trim" : TrimCropper,
        "passthrough" : PassthroughCropper,
    }

    def __init__(self, node_name: str):
        super().__init__(node_name)

        self.load_launch_parameters()

        self._export_dir = self.create_export_dir(self._export_bin)

        # Counter to keep track of image received
        self.img_counter = 0

        # Debug variables for keeping track of the maximum width and height
        # of the received segments/lines.
        self._debug_max_width, self._debug_max_height = (0, 0)

        self.bridge = CvBridge()
        self.cropper = StreamToDatasetNode.CROPPERS[self._crop_type](
            self._thickness, bins=self._dimensions)

        self.sub_ood_input = self.create_subscription(
            DetectorInput,
            "~/ood_input",
            self.cb_ood_input,
            100)

        dimens_debug_str = ""
        for dimen in self._dimensions:
            dimens_debug_str += f"{dimen[0]}x{dimen[1]}, "
        self.get_logger().info(f"Initialized with: \
export_bin: {self._export_bin}, \
single_frame: {self._single_frame}, \
crop_type: {self._crop_type}, \
dimensions: {dimens_debug_str}, \
thickness: {self._thickness}")

        if self._crop_type not in ["bin", "crop"] and len(self._dimensions) > 0:
            self.get_logger().warn(
                f"Ignoring given dimensions with crop type \
'{self._crop_type}'")

    def load_launch_parameters(self):
        self.declare_parameter("export_bin")
        self.declare_parameter("single_frame")
        self.declare_parameter("crop_type")
        self.declare_parameter("dimensions")
        self.declare_parameter("thickness")

        self._export_bin = self.get_parameter("export_bin")\
            .get_parameter_value().string_value
        self._single_frame = self.get_parameter("single_frame")\
            .get_parameter_value().bool_value
        self._crop_type = self.get_parameter("crop_type")\
            .get_parameter_value().string_value
        _dimens = self.get_parameter("dimensions")\
            .get_parameter_value().integer_array_value
        self._dimensions = [(val, val) for val in _dimens]
        self._thickness = self.get_parameter("thickness")\
            .get_parameter_value().integer_value

    def create_export_dir(self, export_bin: str) -> str:
        """Create a directory in the given `export_bin`. All images are
        saved in this new directory.

        :param export_bin: Full path of the export bin.
        :type export_bin: str
        :raises ValueError: Path of the export bin does not exist.
        :return: Full path of the created directory in the export bin.
        :rtype: str
        """
        if not os.path.isdir(export_bin):
            error_msg = "f{export_bin} does not exist"
            raise ValueError(error_msg)

        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        random_dir = os.path.join(
            export_bin, f"{self._crop_type}_{sec}_{nanosec}")
        os.makedirs(random_dir)
        self.get_logger().info(f"Results stored in {random_dir}")

        if self._crop_type == "bin":
            for width, height in self._dimensions:
                bin_dir = os.path.join(random_dir, f"{width}x{height}")
                os.makedirs(bin_dir)

        return random_dir

    def cb_ood_input(self, msg: DetectorInput):
        self.img_counter += 1
        if self.img_counter == 1:
            self.get_logger().info("Received first image!")
        # Don't process message if only single frame is specified
        if self._single_frame and self.img_counter > 1:
            return
        uncompressed_img = self.bridge.compressed_imgmsg_to_cv2(
            msg.frame, "bgr8")
        lines, color_ids = StreamToDatasetNode._extract_lines(msg.lines)

        for idx, line in enumerate(lines):
            cropped_img = self.cropper.crop_segments(uncompressed_img, line)

            width, height = StreamToDatasetNode._get_line_dimension(line)
            if width > self._debug_max_width:
                 self._debug_max_width = width
            if height > self._debug_max_height:
                 self._debug_max_height = height

            if self._crop_type == "bin":
                cropped_shape = cropped_img.shape
                cropped_height = cropped_shape[0]
                cropped_width = cropped_shape[1]

                file_path = os.path.join(
                    self._export_dir,
                    f"{cropped_width}x{cropped_height}",
                    f"{self.img_counter}_{idx}.png")
            else:
                file_path = os.path.join(
                    self._export_dir, f"{self.img_counter}_{idx}.png")

            cv2.imwrite(file_path, cropped_img)
        
        if self.img_counter == 1 and self._single_frame:
            self.get_logger().warn("Only cropping single frame, \
terminating node")
            raise SystemError

    @staticmethod
    def _get_line_dimension(line) -> Tuple[int, int]:
        """Get the width and height of a line. The width is defined as the
        distance between 2 x-coordinates, and height is the distance between
        2 y-coordinates.

        :param line: _description_
        :type line: _type_
        :return: Tuple where the elements are (width, height).
        :rtype: Tuple[int, int]
        """
        return abs(line[2] - line[0]), abs(line[3] - line[1])

    @staticmethod
    def _extract_lines(lines):
        """Utility function for extracting segments/lines.
        """
        xys = []
        color_ids = []
        for line in lines:
            p1 = line.coordinates[0]
            p2 = line.coordinates[1]
            xys.append([p1.x, p1.y, p2.x, p2.y])
            color_ids.append(line.color)
        return np.array(xys, np.int), color_ids

    def cleanup(self):
        self.get_logger().info(f"Max width: {self._debug_max_width}, \
max height: {self._debug_max_height}")


def main(args=None):
    rclpy.init(args=args)
    node = StreamToDatasetNode("stream_to_dataset_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemError:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
