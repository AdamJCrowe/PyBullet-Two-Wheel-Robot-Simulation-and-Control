import time
import numpy as np

from robot_system import LQRController, Observer, Encoder, IMU, Actuator
from utils import Simulator, Logger
from config import sim_setup, sim_fine_tune, lqr_weights, inertia_params, imu_params, encoder_params, actuator_params


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
        simulator.save_state()
        
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
    


