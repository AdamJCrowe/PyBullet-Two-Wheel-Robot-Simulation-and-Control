import control
import numpy as np


class LQRController:
    """
    Linear Quadratic Regulator (LQR) controller based on planar dynamics for two-wheel robot
    Q matrix is weighting on minimising state variables (wheel pos, wheel vel, body angle, body vel) [x]
    R matrix is weighting on minimising actuator torque [u]
    K matrix is gains which correlate state variable error to torque [u=-Kx]
    Controllability is assessed based on the condition number
    """
    
    def __init__(self, utils, **kwargs):
        self.utils = utils
        
        for key, value in kwargs.items():
            setattr(self, key, value)
            
        self.calculate_gains()

        self.assess_controllability()


    def calculate_gains(self):
        # system dynamics constants
        a = 2*self.utils.wheel_mass*(self.utils.wheel_radius**2) +2* self.utils.body_MoI_wheel_axis + self.utils.body_mass * (self.utils.wheel_radius**2)
        b = self.utils.body_mass * self.utils.wheel_base_offset * self.utils.wheel_radius
        c = self.utils.body_mass * (self.utils.wheel_base_offset**2) + self.utils.body_MoI_wheel_axis
        d = self.utils.body_mass *9.81* self.utils.wheel_base_offset

        # A and B matricies
        A32 = (d/b) + ((a*c*d) / (b**3 - a*b*c))
        A34 = -(a*d) / (b**2 - a*c)
        B12 = -(2*c) / (b**2 - a*c)
        B14 = (2*b) / (b**2 - a*c)
        n=4
        self.A = np.array(([0,1,0,0], [0,0,A32,0], [0,0,0,1], [0,0,A34,0]))
        self.B = np.array(([0],[B12],[0],[B14]))

        # LQR weighted gain matricies
        LQR_Q = np.array(([self.wheel_pos,0,0,0], [0,self.wheel_vel,0,0], [0,0,self.body_angle,0], [0,0,0,self.body_vel])) 
        LQR_R = np.array(([self.torque])) 

        self.K, _, _ = control.lqr(self.A, self.B, LQR_Q, LQR_R) # K is state feedback gains, S is solution to Ricatti equation and E eigenvalues
        print("LQR gain constants: ", self.K)
        self.K = np.array(([-self.K[0,0], -self.K[0,1], self.K[0,2], self.K[0,3]])) # wheel's rotate opposite to EoM diagram


    def assess_controllability(self):
        Ctrb = control.ctrb(self.A,self.B)
        

        CtrbT = np.transpose(Ctrb)
        Gram = np.dot(Ctrb,CtrbT)
        n=4
        for i in range(n):
            for j in range(n):
                Gram[i, j] = Gram[i, j] / 1000000 # adjust to allow SVD calcs
        U, S, V = np.linalg.svd(Gram) # U*S*VT should equal Gram
        VT = np.transpose(V)
        sigma = np.zeros((n, n))
        for i in range(n):
            sigma[i, i] = S[i]
        USVT = np.dot(U, np.dot(sigma,VT))
        if (np.allclose(Gram, USVT) == True):       
            print("Controller's condition number is (where smaller is more controllable):", np.linalg.cond(sigma))

