"""Camera frames as plain RGB bytes for the window (spec 2026-11-11, 5). numpy only: no ROS, no Qt."""
import numpy as np

# encoding -> (channels, the indices of R, G, B in a pixel)
LAYOUTS = {'rgb8': (3, (0, 1, 2)), 'bgr8': (3, (2, 1, 0)), 'rgba8': (4, (0, 1, 2)), 'bgra8': (4, (2, 1, 0)),
           'mono8': (1, (0, 0, 0))}


def to_rgb(encoding, width, height, step, data):
    """width * height * 3 bytes of RGB, rows packed, from the fields of a sensor_msgs/Image.

    step is the length of a row in bytes and may include padding. None for an encoding that is not known or a
    buffer that does not hold the image it claims.
    """
    layout = LAYOUTS.get(encoding)
    if layout is None or width <= 0 or height <= 0:
        return None
    channels, order = layout
    if step < width * channels:
        return None
    pixels = np.frombuffer(data, dtype=np.uint8)  # bytes, bytearray or the array('B') rclpy hands over
    if pixels.size < step * height:
        return None
    rows = pixels[:step * height].reshape(height, step)[:, :width * channels].reshape(height, width, channels)
    return rows[:, :, list(order)].tobytes()
