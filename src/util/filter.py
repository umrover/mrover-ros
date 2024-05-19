import numpy as np
from typing import Union, Optional


class SinglePoleLowPass:
    """
    Infinite impulse response first order lowpass filter.

    Tau: RC time constant
    dt: signal update frequency
    """

    def __init__(self, tau: float, dt: float):
        self.y: Optional[Union[np.ndarray, float]] = None
        self.alpha = dt / (tau + dt)

    def update(self, x: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        if self.y is None:
            self.y = x
        else:
            self.y = self.alpha * x + (1 - self.alpha) * self.y
        return self.y
