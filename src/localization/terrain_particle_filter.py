#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt


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
        sigma = 0.0
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
            x1, y1, theta1, _ = p
            
            # add noise to the deltas
            x2n = x2 + np.random.default_rng().normal(0, self.sigma_x)
            theta2n = theta2 + np.random.default_rng().normal(0, self.sigma_theta)

            dx = x2n * np.cos(theta1) - y2 * np.sin(theta1)
            dy = y2 * np.cos(theta1) + x2n * np.sin(theta1) 
            dtheta = theta2n
            p += np.array([dx, dy, dtheta, 0])
            
    
class TerrainParticleFilterNode:
    def __init__(self) -> None:
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        rospy.Subscriber('/ground_truth', Odometry, self.ground_truth_callback)
        self.pf = TerrainParticleFilter(None, 30, np.array([0, 0, 0]))
        self.last_time = rospy.Time.now()
        self.gt_count = 10
        self.pf_list = []
        self.gt_list = []
        rospy.on_shutdown(self.plot_data)
    
    def cmd_callback(self, msg: Twist) -> None:
        cur_time = rospy.Time.now()
        dt = (cur_time - self.last_time).to_sec()
        self.pf.predict(np.array([msg.linear.x, msg.linear.y, msg.angular.z]), dt)
        self.last_time = cur_time
        
    def ground_truth_callback(self, msg: Odometry) -> None:
        if self.gt_count < 2000:
            self.gt_count += 1
            return 
        self.pf_list.append(self.pf.particles[:, :2])
        self.gt_list.append(np.array([msg.pose.pose.position.x, msg.pose.pose.position.y]))
        self.gt_count = 0
        
    
    def plot_data(self) -> None:
        self.pf_data = np.vstack(self.pf_list)
        self.gt_data = np.vstack(self.gt_list)
        self.gt_data -= self.gt_data[0]
        plt.plot(self.pf_data[:, 0], self.pf_data[:, 1], 'r.', label="Particle Filter")
        plt.plot(self.gt_data[:, 0], self.gt_data[:, 1], 'b.', label="Ground Truth")
        plt.show()

if __name__ == '__main__':
    rospy.init_node('terrain_particle_filter')
    TerrainParticleFilterNode()
    rospy.spin()
            




















