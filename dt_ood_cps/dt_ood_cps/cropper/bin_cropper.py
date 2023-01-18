#!/usr/bin/env python3

import cv2
import numpy as np

from typing import List, Tuple

from dt_ood_cps.cropper import (
    FitCropper,
    ISegmentCropper
)


__all__ = ['BinCropper']



class BinCropper(ISegmentCropper):
    """Size of output images fits around the segment, hence depends on the
    segment size. Segment is extracted as it. The image is then resized to the
    closest bin.
    """
    def __init__(self, thickness: int = 2, **kwargs):
        """
        :param thickness: Thickness of the cropped line segment, defaults to 2
        :type thickness: int, optional
        """
        super().__init__(thickness, **kwargs)
        _bins: List[Tuple[int,int]] = kwargs["bins"]

        assert(len(_bins) >= 1)
        for bin in _bins:
            # Assume square images
            if bin[0] != bin[1]:
                raise NotImplementedError("Only square images supported")
        self.bins = list([bin[0] for bin in _bins])

        # Check if bin sizes are in ascending order
        assert(all(self.bins[i] <= self.bins[i+1] for i in
            range(len(self.bins) - 1)))

        self.fit_cropper = FitCropper(thickness)

    def crop_segments(self, frame, line: np.array):
        cropped_frame = self.fit_cropper.crop_segments(frame, line)
        shape = cropped_frame.shape
        height = shape[0]
        width = shape[1]

        larger_dim = height if height >= width else width
        larger_bin = np.digitize(larger_dim, self.bins)
        if larger_bin == len(self.bins):
            larger_bin -= 1

        return cv2.resize(cropped_frame, (self.bins[larger_bin],
            self.bins[larger_bin]), cv2.INTER_CUBIC)


if __name__ == "__main__":
    frame = np.full((360,480,3), (192,192,192), dtype=np.uint8)
    p1 = (123, 81)
    p2 = (350, 256)
    p1 = (3, 3)
    p2 = (25, 0)

    # p1 = (198, 47)
    # p2 = (158, 44)

    print(f"{abs(p1[0]-p2[0]), abs(p1[1]-p2[1])}")

    frame = cv2.line(frame, p1, p2, (0,0,255), 10)
    frame = cv2.line(frame, (20,20), (400,400), (255,0,0), 10)
    frame = cv2.circle(frame, (400,100), 50, (255,255,255), -1)
    frame = cv2.circle(frame, p1, 4, (255,255,255), -1)
    frame = cv2.circle(frame, p2, 4, (255,255,255), -1)

    print(f"Original image dimension: {frame.shape}")
    cv2.imshow('Orginal', frame)
    cv2.waitKey(0)

    cropper = BinCropper(bins=[(16,16),(32,32),(64,64)])
    cropped_frame = cropper.crop_segments(frame, [p1[0], p1[1], p2[0], p2[1]])

    print(f"Cropped image dimension: {cropped_frame.shape}")
    cv2.imshow('Cropped', cropped_frame)
    cv2.waitKey(0)
