#!/usr/bin/env python3

import numpy as np

from abc import ABC, abstractmethod


__all__ = ['ISegmentCropper']


class ISegmentCropper(ABC):
    """Interface for segment croppers."""
    def __init__(self, thickness: int=2, **kwargs) -> None:
        self._thickness = thickness
        super().__init__()

    @abstractmethod
    def crop_segments(self, frame, line: np.array):
        """Crop an image using the a single bounded lines."""
        pass
