#!/usr/bin/env python3

import cv2
import numpy as np

from typing import Tuple

from dt_ood_cps.cropper import (
    FitCropper,
    ISegmentCropper
)


__all__ = ['TrimCropper']


class TrimCropper(ISegmentCropper):
    """Size of the output image is fixed. If the segment is larger than can fit
    inside the image, its edges are cropped/removed/trimmed off.
    """
    def __init__(self, thickness: int = 2, **kwargs):
        """
        :param thickness: Thickness of the cropped line segment, defaults to 2
        :type thickness: int, optional
        """
        super().__init__(thickness, **kwargs)
        _dimen: Tuple[int,int] = kwargs["bins"][0]
        if 0 in _dimen:
            raise ValueError("Dimension cannot be 0")
        if _dimen[0] != _dimen[1]:
            raise NotImplementedError("Only square images supported")
        self.dimension = _dimen[0]

        self.fit_cropper = FitCropper(thickness)

    def pad_frame(self, frame):
        """Pad the image to ensure that both dimensions are at least as large
        as what is needed. Padding is done to ensure that the original image
        is in the middle.
        """
        shape = frame.shape
        height = shape[0]
        width = shape[1]

        # Image is greater than the desired dimension, so no need to pad
        if height >= self.dimension and width >= self.dimension:
            return frame

        # If both dimensions are smaller than the desired dimension, then we
        # just superimpose the image on a black background
        if height < self.dimension and width < self.dimension:
            black_bg = np.full((self.dimension,self.dimension,3), (0,0,0),
                dtype=np.uint8)
            yoff = (self.dimension-height)//2
            xoff = (self.dimension-width)//2
            # Superimpose
            black_bg[yoff:yoff+height,xoff:xoff+width] = frame
            return black_bg

        # If only 1 dimension is smaller than the desired dimension, then we
        # pad only that dimension and leave the other dimension intact.
        larger_dimen = height if height > width else width
        black_bg = np.full((larger_dimen,larger_dimen,3), (0,0,0),
            dtype=np.uint8)
        yoff = (larger_dimen-height)//2
        xoff = (larger_dimen-width)//2
        # Superimpose
        black_bg[yoff:yoff+height,xoff:xoff+width] = frame
        return black_bg

    def trim_frame(self, frame):
        center = frame.shape
        x = center[1]/2 - self.dimension/2
        y = center[0]/2 - self.dimension/2

        return frame[int(y):int(y+self.dimension), int(x):int(x+self.dimension)]


    def crop_segments(self, frame, line: np.array):
        cropped_frame = self.fit_cropper.crop_segments(frame, line)

        # If at least 1 dimension of the segment is smaller than the desired
        # size, then we pad that dimension
        return self.trim_frame(self.pad_frame(cropped_frame))


if __name__ == "__main__":
    frame = np.full((360,480,3), (192,192,192), dtype=np.uint8)
    p1 = (150, 80)
    p2 = (310, 180)
    print(f"{abs(p1[0]-p2[0]), abs(p1[1]-p2[1])}")

    frame = cv2.line(frame, p1, p2, (0,0,255), 10)
    frame = cv2.line(frame, (20,20), (400,400), (255,0,0), 10)
    frame = cv2.circle(frame, (400,100), 50, (255,255,255), -1)
    frame = cv2.circle(frame, p1, 4, (255,255,255), -1)
    frame = cv2.circle(frame, p2, 4, (255,255,255), -1)
    frame = cv2.circle(frame, ((p1[0]+p2[0])//2,(p1[1]+p2[1])//2), 4,
        (255,255,255), -1)

    print(f"Original image dimension: {frame.shape}")
    cv2.imshow('Orginal', frame)
    cv2.waitKey(0)

    cropper = TrimCropper(thickness=3, bins=[(128,128)])
    cropped_frame = cropper.crop_segments(frame, [p1[0], p1[1], p2[0], p2[1]])

    print(f"Cropped image dimension: {cropped_frame.shape}")
    cv2.imshow('Cropped', cropped_frame)
    cv2.waitKey(0)
