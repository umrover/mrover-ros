from dataclasses import dataclass, field
from enum import Enum
from math import copysign


@dataclass
class DeviceInputs:
    axes: list[float] = field(default_factory=list)
    buttons: list[float] = field(default_factory=list)


def remap(value: float, old_min: float, old_max: float, new_min: float, new_max: float) -> float:
    return (value - old_min) / (old_max - old_min) * (new_max - new_min) + new_min


def deadzone_filter(signal: float, threshold: float) -> float:
    """
    Values lower than the threshold will be clipped to zero.
    Those above will be remapped to [0, 1] linearly.
    """
    magnitude = abs(signal)
    magnitude = 0 if magnitude < threshold else remap(magnitude, threshold, 1, 0, 1)
    return copysign(magnitude, signal)


def quadratic_filter(signal: float) -> float:
    """
    Use to allow more control near low inputs values by squaring the magnitude.
    For example using a joystick to control drive.
    """
    return copysign(signal**2, signal)


def filter_input(value: float, deadzone: float = 0.1, quadratic: bool = False, scale: float = 1.0) -> float:
    """
    Filter the input signal with a deadzone and quadratic scaling.
    """
    value = deadzone_filter(value, deadzone)
    if quadratic:
        value = quadratic_filter(value)
    value *= scale
    return value


def safe_index(values: list[float], index: Enum) -> float:
    return values[index.value] if values else 0.0


def simulated_axis(axes: list[float], positive_index: Enum, negative_index: Enum) -> float:
    """
    Simulate a single axis from two buttons.
    """
    return safe_index(axes, positive_index) - safe_index(axes, negative_index)
