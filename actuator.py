import math


class Actuator:
    def __init__ (self, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

        self.spd_trq_const = -self.max_torque / self.max_speed
        self.torque=0.0


    def torque_ripple(self, wheel_pos):
        ripple_offset = self.torque_ripple_amplitude * math.sin(wheel_pos * self.commutations)
        self.torque = self.torque *(1 + ripple_offset/100)


    def back_emf(self, wheel_vel):
        torque_limit = self.max_torque - abs(self.spd_trq_const * wheel_vel) 
        if (self.torque >= torque_limit):
            self.torque = torque_limit
        elif (self.torque <= -torque_limit):
            self.torque = -torque_limit
