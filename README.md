# PyBullet-Two-Wheel-Robot-Simulation-and-Control
- PyBullet Simulation of a two-wheel robot that is controlled using an LQR controller.
- Simulation accuracy confirmed by testing a physical prototype.
- Angle, velocity and motor torque data is logged and plotted.

## Simulation incudes
- Disturbances
- Motor back-emf and torque ripple
- IMU drift and noise
- Encoder resolution
- Gearbox friction

## Simulation setup
1. URDF file: specifies the robot's dimensions and inertia terms
2. Config file -> parameters: specifies hardware properties such as max motor torque and encoder resolution
3. Config file -> simulation settings: specifies disturbances, run time, starting pose, etc
4. Config file -> LQR weights: tuning the controller

## Future plans
1. Optimise LQR controller
2. Implement Kalman filter
3. Add motion planning/tracking using linear MPC
