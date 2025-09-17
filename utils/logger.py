import numpy as np
import matplotlib.pyplot as plt

class Logger:
    """
    Log and plot true/measured state and torque
    """
    
    def __init__(self, total_steps):
        self.states = np.zeros((4, total_steps),dtype=float)
        self.observed_states = np.zeros((4, total_steps + 1),dtype=float)
        self.actuator_torques = np.zeros((1, total_steps),dtype=float)


    def save_data(self, i, save_state, save_measured_state, save_actuator_torque):
        self.states[:,i] = save_state
        self.observed_states[:,i] = save_measured_state
        self.actuator_torques[0,i] = save_actuator_torque


    def plot_data(self, total_time, total_steps, position_plot_data, velocity_plot_data, start_position):
        self.time_steps = np.linspace(0, total_time, total_steps)
        fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

        wheel_plots = []
        body_plots = []
        
        if (position_plot_data == True):
            wheel_plots.extend([
                ("Wheel Angle",          self.states[0, :],       "b", "pos"),
                ("Measured Wheel Angle",    self.observed_states[0,1:], "b", "measured")
            ])
            body_plots.extend([
                ("Body Angle",          self.states[2, :],       "b", "pos"),
                ("Measured Body Angle", self.observed_states[2,1:], "b", "measured")
                
            ])

        if (velocity_plot_data == True):
            wheel_plots.extend([
                ("Measured Wheel Velocity", self.observed_states[1,1:], "r", "measured"),
                ("Wheel Velocity",       self.states[1, :],       "r", "vel")
            ])
            body_plots.extend([
                ("Body Velocity",       self.states[3, :],       "r", "vel"),
                ("Measured Body Velocity", self.observed_states[3,1:], "r", "measured")
            ])
                          
        actuator_plots = [
            ("Actuator Torque",          self.actuator_torques.flatten(),       "b", "torque")
        ]
        
        plot_specs = [wheel_plots, body_plots, actuator_plots]

        for i, plots in enumerate(plot_specs):
            base_ax = axs[i]        
            current_ax = base_ax
            for j, (label, data, color, twinx_type) in enumerate(plots):
                if j == 0:
                    current_ax = base_ax
                else:
                    current_ax = base_ax.twinx()
                self.add_to_plot(current_ax, label, data, color, twinx_type)

        plt.tight_layout(pad=3.0)
        plt.show()


    def add_to_plot(self, variable, set_label, dataset, set_colour, mode):
        if mode == "pos":
            label_data = f"True/Measured \n {set_label} [rad]"
            variable.set_ylabel(label_data, color=set_colour)
            plot_colour = f"{set_colour}-"
            variable.grid(True)
        elif mode == "torque":
            label_data = f"True {set_label} [Nm]"
            variable.set_ylabel(label_data, color=set_colour)
            plot_colour = f"{set_colour}-"
            variable.grid(True)
        elif mode == "vel":
            label_data = f"True/Measured \n {set_label} [rad/s]"
            variable.set_ylabel(label_data, color=set_colour)
            plot_colour = f"{set_colour}-"
            variable.spines["right"]
        elif mode == "measured":
            plot_colour = f"{set_colour}--"
            variable.yaxis.set_ticks([])

        variable.plot(self.time_steps, dataset, plot_colour, label=set_label)
        variable.tick_params(axis="y", labelcolor=set_colour)
            


