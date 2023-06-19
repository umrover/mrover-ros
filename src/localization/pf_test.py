import numpy as np
import matplotlib.pyplot as plt
# from mpl_toolkits import mplot3d
from terrain_particle_filter import TerrainParticleFilter
import rasterio
import cv2


def test_predict():
    pf = TerrainParticleFilter(None, 1, 0.0, 0.0, np.array([0.0, 0.0, 0.0]))
    pf_list = []

    for i in range(1000):
        pf_list.append(np.copy(pf.particles[:, :2]))
        pf.predict(np.array([0.1, 0.0, -1.2]), 0.01)

    pf_data = np.vstack(pf_list)
    print(pf_data.shape)
    plt.plot(pf_data[:, 0], pf_data[:, 1], "r-", label="Particle Filter")
    plt.show()


def test_terrain():
    length = 50
    width = 50
    height = 4.820803273566
    
    with rasterio.open("terrain.tif") as f:
        # <pos>0 0 0</pos>
        # <size>50 50 4.820803273566</size>
        terrain_map = f.read().squeeze()
    p_length = length / terrain_map.shape[0]
    p_width = width / terrain_map.shape[1]
    print(f"vmin: {np.min(terrain_map)}, vmax: {np.max(terrain_map)}")
    
    points = []
    for i in range(terrain_map.shape[0]):
        for j in range(terrain_map.shape[1]):
            x = (i + 0.5) * p_length
            y = (j + 0.5) * p_width
            z = terrain_map[i, j] #* height
            points.append(np.array([x, y, z]))
    points = np.vstack(points)
    
    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c=points[:, 2], marker=".", s=0.1)
    plt.show()


if __name__ == "__main__":
    test_terrain()
