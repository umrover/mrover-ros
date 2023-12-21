# TODO
- add manif to cmake


# Invariant EKF
Kalman Filter that uses lie group operations for filtering. Should take in some combination of the following: orientation, accelerometer, gyro, magnetometer, GPS, GPS heading, wheel encoders, and throttle command; should output pose and probably velocity.

### Math
- read invariant EKF papers
- choose variables and representation
    - SE_2(3): pose and linear velocity
    - might try to add angular velocity later
- figure out rover dynamics equations
    - just use kinematics from IMU measurements:
    $$ 
        \begin{align*}
        p_{k+1} &= p_k + v_k \delta t + a_{k+1} \frac{\delta t^2}{2} \\
        v_{k+1} &= v_k + a_{k+1} \delta t \\
        R_{k+1} &= R\exp([\omega]_\times)
        \end{align*}
    $$
- figure out general invariant EKF equations
    - Let $\mathcal{\hat{X}}_k \in \mathcal{M}$ be the state estimate at time $k$, $P_k \in \mathbb{R}^{n \times n}$ be the state covariance matrix (defined in the tangent space) at time $k$, and $\textbf{u}_k \in \mathbb{R}^n \cong T_{\mathcal{X}}\mathcal{M}$ be the tangent space vector representing an increment to the state at time $k$:
$$
    \begin{align*}
        \hat{\mathcal{X}}_{k+1|k} &= f(\hat{\mathcal{X}}_{k|k}, \textbf{u}_{k+1}) = \hat{\mathcal{X}}_{k|k} \oplus \textbf{u}_{k+1} \\
        P_{k+1|k} &= F_{k+1}P_{k|k}F_{k+1}^\top + G_{k+1}QG_{k}^\top \\
         & F_{k+1} = J_{\hat{\mathcal{X}}_{k|k}}^{\hat{\mathcal{X}}_{k+1|k}}, \ G_{k+1} = J_{\textbf{u}_{k+1}}^{\hat{\mathcal{X}}_{k+1|k}} \\
        K_{k+1} &= P_{k+1|k} H_{k+1}^\top (H_{k+1}P_{k+1|k}H_{k+1}^\top + R)^{-1} \\
        & H_{k+1} = J_{\hat{\mathcal{X}}_{k+1|k}}^{\hat{\textbf{y}}_{k+1}} = J_{\hat{\mathcal{X}}_{k+1|k}}^{g(\hat{\mathcal{X}}_{k+1|k})} \\
        \hat{\mathcal{X}}_{k+1|k+1} &= \hat{\mathcal{X}}_{k+1|k} \oplus (\textbf{y}_{k+1} - \hat{\textbf{y}}_{k}) \\
    P_{k+1|k+1} &= P_{k+1|k} - P_{k+1|k}H_{k+1}^\top (H_{k+1}P_{k+1|k}H_{k+1}^\top + R)^{-1}
    \end{align*}
$$
- choose prediction and measurement inputs/sensors
    - accel and gyro for prediction
    - GPS, mag, accel for correction
- develop equations for prediction step and each measurement step for specific sensors
    - prediction step:
    $$
    \begin{align*}
        \mathcal{X}_k &= \begin{bmatrix}
        R_k & p_k & v_k \\
        0 & 1 & 0 \\
        0 & 0 & 1
        \end{bmatrix}
        \in \text{SE}_2(3)
    \end{align*}
    $$
    Where $R \in \mathbb{R}^{3 \times 3}$ is a rotation matrix representing the orientation of the robot body frame relative to some fixed world frame, $p \in \mathbb{R}^3$ is a vector representing the position of the robot in the world frame, and $v \in \mathbb{R}^3$ is a vector representing the velocity of the robot in the world frame.
    $$
    \textbf{u}_k = \begin{bmatrix}
        a_k \\ \omega_k
    \end{bmatrix}, \ a = \begin{bmatrix} a_x \\ a_y \\ a_z \end{bmatrix}, \ \omega = \begin{bmatrix} \omega_x \\ \omega_y \\ \omega_z \end{bmatrix}
    $$
    Where $a_i$ and $\omega_i$ are respectively the measured linear acceleration and angular velocity along the $i$ th axis, with respect to the body frame.
    $$
    \begin{align*}
        g &= \begin{bmatrix} 0 \\ 0 \\ -9.81 \end{bmatrix}, \ a_{adj} = a + R^\top g \\
        \delta p &= R^\top v \delta t + \frac{1}{2}a_{adj} \delta t^2 \\
        \delta R &= \omega \delta t \\
        \delta v &= a_{adj} \delta t \\
        \hat{\mathcal{X}}_{k+1|k} &= \hat{\mathcal{X}}_{k|k} \oplus u_k, \ 
        u = \begin{bmatrix} \delta p \\ \delta R \\ \delta v \end{bmatrix} \\
        F_{k+1} &= J_{\hat{\mathcal{X}}_{k|k}}
    \end{align*}
    $$

### EKF Class Implementation
- choose lie group library (manif, smooth, etc)
    - start with manif, try out smooth later
- figure out storage and management of covariance matrices
    - constructor should accept default covariances
    - each update step should accept an optional covariance matrix to override the default
- implement functions for prediction step and measurement steps
- add debug output
- should it be templated?

### ROS Node and Infrastructure
- subscribers to all sensors
- publish pose message
- publish twist
- publish pose to TF tree
- configure covariances and other parameters with rosparam
- rate profiling?

### Testing in sim
- make sure all simulated sensors are properly configured
- set up automated test trajectory
- compare to no filtering and to existing EKF
- possibly walk through trajectory step by step to ensure equations are behaving correctly
