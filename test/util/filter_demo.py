from util.filter import SinglePoleLowPass
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Simple test case
    t = np.linspace(0, 3 * np.pi)
    x = np.sin(t) + 0.3 * np.random.rand(len(t))
    f = SinglePoleLowPass(0.5, t[1])
    y = [f.update(v) for v in x]
    plt.plot(t, x)
    plt.plot(t, y)
    plt.show()
