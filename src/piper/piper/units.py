"""PiPER joint wire units are signed integers in 0.001 degrees."""
import math

RAD_PER_MILLIDEGREE = math.pi / 180000.0
MILLIDEGREES_PER_RAD = 180000.0 / math.pi


def to_millidegrees(radians):
    if not math.isfinite(radians):
        raise ValueError('Joint command must be finite')
    return round(radians * MILLIDEGREES_PER_RAD)


def to_radians(millidegrees):
    return millidegrees * RAD_PER_MILLIDEGREE
