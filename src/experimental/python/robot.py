from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple, Union

import numpy as np


@dataclass # type: ignore
class Robot(ABC):
    """
    Generic robot dynamics model x' = f(x, u)
    """

    x: np.ndarray

    @abstractmethod
    def update_state(self, u: np.ndarray, dt: float) -> np.ndarray:
        """
        Updates the internal state and returns the next state via
        whatever numerical ODE method is chosen

        Args:
        u - control input
        dt - elapsed time since previous polling
        """

    @abstractmethod
    def get_drawable(self) -> Tuple[np.ndarray, float]:
        """ Our sim can draw a 2d robot with a position and orientation
        """


@dataclass # type: ignore
class LinearRobot(Robot):
    """
    Discrete time LTI dynamics model x' = Ax + Bu
    """

    A: np.ndarray
    B: np.ndarray

    def update_state(self, u: np.ndarray, _) -> np.ndarray:
        """
        Args:
        u - state space control input
        """
        return self.A @ self.x + self.B @ u


@dataclass
class UnicycleKinematics(Robot):
    """
    The continious time unicycle kinematics are as follows.

    x' = v cos (theta)
    y' = v sin (theta)
    theta' = omega

    With control inputs: v, omega. 
    """

    def get_drawable(self) -> Tuple[np.ndarray, float]:
        return self.x[:2], self.x[2]

    def update_state(self, u: np.ndarray, dt: float, noise_cov: Union[np.ndarray, None] = None) -> np.ndarray:
        """
        Provides a first-order discrete time Euler integration update
        Optionally introduce additive zero mean white gaussian noise 
        """
        if noise_cov is None:
            noise_cov = np.zeros((3, 3))
        w = np.random.multivariate_normal(np.zeros(3), noise_cov)

        self.x[0] += u[0] * np.cos(self.x[2]) * dt
        self.x[1] += u[0] * np.sin(self.x[2]) * dt
        self.x[2] += u[1] * dt
        self.x += w
        return self.x


@dataclass
class LinearUnicycleKinematics(LinearRobot):
    """
    Linearization of unicycle kinematics
    """

    def get_drawable(self) -> Tuple[np.ndarray, float]:
        return self.x[:2], self.x[2]

    @classmethod
    def from_dt(cls, x: np.ndarray, u: np.ndarray, dt: float) -> LinearUnicycleKinematics:
        A = np.array([[1, 0, -u[0] * np.sin(x[2]) * dt], [0, 1, u[0] * np.cos(x[2]) * dt], [0, 0, 1]])
        B = np.array([[np.cos(x[2]) * dt, 0], [np.sin(x[2]) * dt, 0], [0, dt]])
        return LinearUnicycleKinematics(x, A, B)
