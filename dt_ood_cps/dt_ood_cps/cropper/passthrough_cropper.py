#!/usr/bin/env python3

import cv2
import numpy as np

from dt_ood_cps.cropper import ISegmentCropper


__all__ = ['PassthroughCropper']


class PassthroughCropper(ISegmentCropper):
    """Size of output images is the same as the passed-in image.
    Segment is extracted as is. 
    """
    def __init__(self, thickness: int = 2, **kwargs):
        """
        :param thickness: Thickness of the cropped line segment, defaults to 2
        :type thickness: int, optional
        """
        super().__init__(thickness, **kwargs)

    def crop_segments(self, frame, line: np.array):
        background = np.zeros_like(frame)
        background = cv2.line(
            background, (line[0], line[1]), (line[2], line[3]),
            color=(255,255,255), thickness=self._thickness)
        return cv2.bitwise_and(background, frame)


if __name__ == "__main__":
    frame = np.full((360,480,3), (192,192,192), dtype=np.uint8)
    p1 = (123, 81)
    p2 = (245, 210)
    frame = cv2.line(frame, p1, p2, (0,0,255), 10)
    frame = cv2.line(frame, (20,20), (400,400), (255,0,0), 10)
    frame = cv2.circle(frame, (400,100), 50, (255,255,255), -1)
    frame = cv2.circle(frame, p1, 4, (255,255,255), -1)
    frame = cv2.circle(frame, p2, 4, (255,255,255), -1)

    print(f"Original image dimension: {frame.shape}")
    cv2.imshow('Orginal', frame)
    cv2.waitKey(0)

    cropper = PassthroughCropper()
    cropped_frame = cropper.crop_segments(frame, [p1[0], p1[1], p2[0], p2[1]])

    print(f"Cropped image dimension: {cropped_frame.shape}")
    cv2.imshow('Cropped', cropped_frame)
    cv2.waitKey(0)
