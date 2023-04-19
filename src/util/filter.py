import numpy as np
from typing import Union, Optional
import matplotlib.pyplot as plt


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


if __name__ == "__main__":
    # Simple test case
    t = np.linspace(0, 3 * np.pi)
    x = np.sin(t) + 0.3 * np.random.rand(len(t))
    f = SinglePoleLowPass(0.5, t[1])
    y = [f.update(v) for v in x]
    plt.plot(t, x)
    plt.plot(t, y)
    plt.show()
