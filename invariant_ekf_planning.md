# TODO
- add manif to cmake
- 


# Invariant EKF
Kalman Filter that uses lie group operations for filtering. Should take in some combination of the following: orientation, accelerometer, gyro, magnetometer, GPS, GPS heading, wheel encoders, and throttle command; should output pose and probably velocity.

### Math
- read invariant EKF papers
- choose state variables and representation
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
- choose prediction and measurement inputs/sensors
    - accel and gyro for prediction
    - GPS, mag, accel for correction
- develop equations for prediction step and each measurement step for specific sensors

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
