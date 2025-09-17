# PyBullet-Two-Wheel-Robot-Simulation-and-Control
PyBullet Simulation of a two-wheel robot that is controlled using an LQR controller. Simulation accuracy confirmed by testing a physical prototype.

## Simulation incudes
- Disturbances
- Motor back-emf and torque ripple
- IMU drift and noise
- Encoder resolution
- Gearbox friction

## Simulation setup
1. URDF file: specifies the robot's dimensions and inertia terms
2. Main script -> parameters: specifies hardware properties such as max motor torque and encoder resolution
3. Main script -> simulation settings: specifies disturbances, run time, starting pose, etc
4. Main script -> LQR weights: tuning the controller

## Future plans
1. Optimise LQR controller
2. Implement Kalman filter
3. Add motion planning/tracking using linear MPC
