import numpy as np
import matplotlib.pyplot as plt
import rasterio

x = np.linspace(0, 25, 255)
y = np.linspace(-12.5, 12.5, 255)
X, Y = np.meshgrid(x, y)
Z = np.sin(1.5*np.sqrt(X**2 + Y**2)) + 1

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.set_zlim(0, 10)
surf = ax.plot_surface(X, Y, Z, cmap=plt.cm.coolwarm, linewidth=0, antialiased=True)
plt.show()

with rasterio.open('waves.tif',
                        'w',
                        driver='GTiff',
                        height=Z.shape[0],
                        width=Z.shape[1],
                        count=1,
                        dtype=Z.dtype,
) as dataset:
    dataset.write(Z, 1)