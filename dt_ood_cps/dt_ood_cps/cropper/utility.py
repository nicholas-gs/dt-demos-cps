#!/usr/bin/env python3

from typing import Tuple


__all__ = ['clamp_coordinates']


def clamp_coordinates(v1: int, v2: int, max: int) -> Tuple[int, int]:
    """An image cannot have a shape of 0 in any dimension. So if the
    coordinates are equal, add a 1 pixel buffer.

    :param v1: _description_
    :type v1: int
    :param v2: _description_
    :type v2: int
    :param max: _description_
    :type max: int
    :return: _description_
    :rtype: Tuple[int, int]
    """
    assert(v1 <= v2)
    assert(max >= 2)

    # Do nothing
    if v1 < v2:
        return (v1, v2)
    
    # v1 == v2
    if v1 > 0:
        return (v1-1, v2)
    return (v1, v2+1)