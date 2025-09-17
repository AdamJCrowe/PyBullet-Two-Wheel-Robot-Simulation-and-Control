import numpy as np
from scipy.stats import vonmises


class IMU:
    def __init__(self, loop_time, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

        self.phi = np.exp(-loop_time / self.drift_time)
        self.sigma_drift = np.deg2rad(self.drift_variance) * np.sqrt(1 - self.phi**2)
        sigma_noise = np.deg2rad(self.noise_variance)
        self.kappa = 1.0/(sigma_noise**2)
        self.bias = 0.0


    def add_error(self, angle):
        noise = vonmises.rvs(self.kappa)
        self.bias = self.phi * self.bias + np.random.normal(0, self.sigma_drift)
        measured_angle = angle + noise + self.bias
        return measured_angle


class Encoder:
    def __init__(self, **kwargs):      
        for key, value in kwargs.items():
            setattr(self, key, value)

        self.resolution = (2 * np.pi) / self.pulses # rad per pulse
        self.measured_pos = 0
        self.pulse_count = 0


    def count_pulses(self, position):
        self.pulse_count = (position - self.measured_pos) // self.resolution
        self.measured_pos += self.pulse_count * self.resolution
        return self.measured_pos
