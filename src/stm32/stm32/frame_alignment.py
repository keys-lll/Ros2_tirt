"""Planar vector rotation between native STM32 and laser-aligned ROS axes."""
import math


def rotate_xy(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    # Exact cardinal rotations avoid tiny cross-axis values and integer truncation.
    if abs(c) < 1e-12:
        c = 0.0
    if abs(s) < 1e-12:
        s = 0.0
    return c * x - s * y, s * x + c * y
