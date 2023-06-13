import rospy
import numpy as np


class TerrainParticleFilter:

    def __init__(self, terrain_map, num_particles: int, initial_pose: np.array=None) -> None:
        self.terrain_map = terrain_map
        self.particles = self.init_particles(num_particles, initial_pose)
        self.sigma_x = 0.1
        self.sigma_theta = 0.1

    def tf_matrix(self, pose: np.array) -> np.array:
        """
        Compute the transformation matrix for a given pose.
        
        :param pose: np array containing pose [x, y, theta]
        :returns: 3x3 np array transformation matrix
        """
        x, y, theta = pose
        return np.array([[np.cos(theta), -np.sin(theta), x],
                         [np.sin(theta), np.cos(theta), y],
                         [0, 0, 1]]) 
        
    def init_particles(self, num_particles: int, initial_pose: np.array) -> np.array:
        """
        Initialize the particles with a gaussian distribution around the initial pose estimate.
        
        :param num_particles: number of particles
        :param initial_pose: np array containing initial pose estimate [x, y, theta]
        :returns: Nx4 np array of particles [x, y, theta, weight]
        """
        sigma = 0.1
        poses = np.random.default_rng().normal(initial_pose, sigma, (num_particles, 3))
        return np.hstack((poses, np.ones((num_particles, 1)) / num_particles))

    # prediction step
    def predict(self, u: np.array, dt: float) -> None:
        """
        Update the particle positions based on the input command.
        
        :param u: np array of commanded velocities [v_x, v_y, omega]
        :param dt: time step
        """
        # integrate velocities to get change in pose in the frame of the previous pose
        x2, y2, theta2 = u * dt

        
        for p in self.particles:
            x1, y1, theta1 = p
            
            # add noise to the deltas
            x2n = x2 + np.random.default_rng().normal(0, self.sigma_x)
            theta2n = theta2 + np.random.default_rng().normal(0, self.sigma_theta)

            dx = x2n * np.cos(theta1) - y2 * np.sin(theta1)
            dy = y2 * np.cos(theta1) + x2n * np.sin(theta1) 
            dtheta = theta2n
            p += np.array([dx, dy, dtheta])

            




















