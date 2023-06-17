import numpy as np
import matplotlib.pyplot as plt
from terrain_particle_filter import TerrainParticleFilter

pf = TerrainParticleFilter(None, 1, 0.0, 0.0, np.array([0.0, 0.0, 0.0]))
pf_list = []

for i in range(1000):
    pf_list.append(np.copy(pf.particles[:, :2]))
    pf.predict(np.array([0.1, 0.0, -1.2]), 0.01)

pf_data = np.vstack(pf_list)
print(pf_data.shape)
plt.plot(pf_data[:, 0], pf_data[:, 1], "r-", label="Particle Filter")
plt.show()