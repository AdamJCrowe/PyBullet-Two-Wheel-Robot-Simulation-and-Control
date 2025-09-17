sim_setup = {
    "frequency": 500, # control system Hz
    "total_time": 5.0, # seconds
    "start_position": 0.1, # m
    "start_angle": 15.0, # deg
    "sensor_error": True, 
    "actuator_torque_ripple": True, 
    "max_disturbance": 10.0, # +- max force applied near top of robot (0 to turn off)
    "disturbance_period": 0.2 # disturbance is applied halfway through simulation
}

sim_fine_tune = {
    "slow_motion_factor": 1.0, 
    "sub_steps": 4, # sub steps between control loop step
    "solver_itterations": 100, # constraint solver itteration per sim step (50 is default)
    "position_plot_data": True,
    "velocity_plot_data": True
}

lqr_weights = {
    "wheel_pos": 100.0,
    "wheel_vel": 1.0,
    "body_angle": 100.0,
    "body_vel": 1.0,
    "torque": 1.0
}

inertia_params = {
    "wheel_static_CoF": 0.6,
    "wheel_rolling_CoF": 0.05,
    "wheel_base_offset": 0.114, # distance from centre of body to wheel axis
    "body_MoI_wheel_axis": 0.0206 
}

imu_params = {
    "noise_variance": 0.5, # 1-sigma variance in IMU angle due to noise (deg)
    "drift_variance": 2.0, # 1-sigma variance in IMU angle to gyroscopic drift (deg)
    "drift_time": 60.0 # correlation time - how quickly 1-sigma drift is reached on average (s)
}

encoder_params = {
    "pulses": 748.0 # encoder pulses per wheel revolution
}

actuator_params = {
    "max_torque": 1.18, 
    "max_speed": 36.7, 
    "friction_torque": 0.01, 
    "torque_ripple_amplitude": 20.0, # percentage
    "commutations": 6
}
