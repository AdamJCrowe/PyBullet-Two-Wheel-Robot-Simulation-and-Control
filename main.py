import time
import numpy as np

from simulator import Simulator
from logger import Logger
from lqr_controller import LQRController
from observer import Observer
from sensor import Encoder, IMU
from actuator import Actuator


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


def main():
    actuator = Actuator(**actuator_params)
    simulator = Simulator(actuator.friction_torque, **dict(sim_fine_tune, **sim_setup, **inertia_params))
    logger = Logger(simulator.total_steps)
    controller = LQRController(simulator, **lqr_weights)
    observer = Observer()
    imu = IMU(simulator.loop_time, **imu_params)
    encoder = Encoder(**encoder_params)
    
    time.sleep(0.2)
    simulator.GUI_time = time.time()


    for i in range (simulator.total_steps):
        simulator.update_true_state()
        observer.state[0,0] = encoder.count_pulses(simulator.state[0, 0])
        if (i > 0): observer.state[1,0] = (observer.state[0,0] - logger.observed_states[0,i-1]) / simulator.loop_time
        observer.state[2,0] = imu.add_error(simulator.state[2, 0])
        if (simulator.state[2, 0] != 0.0): observer.state[3,0] = (observer.state[2,0] / simulator.state[2, 0]) * simulator.state[3, 0]

        if (simulator.sensor_error == True): actuator.torque = -np.dot(controller.K, observer.state).item() 
        elif (simulator.sensor_error == False): actuator.torque = -np.dot(controller.K, simulator.state).item()

        if (simulator.actuator_torque_ripple == True): actuator.torque_ripple(simulator.state[0,0]) 
        actuator.back_emf(simulator.state[1,0])

        simulator.set_actuator_torque(actuator.torque, 1)
        simulator.set_actuator_torque(actuator.torque, 2)

        logger.save_data(i, simulator.state[:,0], observer.state[:,0], actuator.torque)

        simulator.create_distrubance(i)
        simulator.correct_GUI_time()
        simulator.step()

    simulator.stop()
    logger.plot_data(simulator.total_time, simulator.total_steps, simulator.position_plot_data, simulator.velocity_plot_data, simulator.start_position)


if __name__ == "__main__":
    main()
    


