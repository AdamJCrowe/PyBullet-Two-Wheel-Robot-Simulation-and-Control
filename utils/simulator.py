import time
import math
import os
import numpy as np
import pybullet as pyb
import pybullet_data
from scipy.spatial.transform import Rotation


class Simulator:
    """
    Handles simulating a two-wheel robot over a set period of time
    Frequency determines how often controller/physics update
    Sub-steps update physics in-between controller updates

    Setup simulation:
    - Load URDF
    - Set robot start pose and camera
    - Find inertia terms which are used by the controller and observor
    - Simulate gearbox friciton by setting joints to have opposing torque

    Running simulation:
    - Save current state of the robot (wheel/body angle/velocity)
    - Apply force disturbance to robot body
    - Set actuator torque
    - Ensure GUI plays at real time speed (faster/slower if slow_motion_factor != 1)
    - Step simulation
    """
    
    def __init__(self, actuator_friction_torque, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)

        self.loop_time = 1./float(self.frequency)
        self.total_steps = int(self.frequency * self.total_time)
        self.state = np.array(([0.0],[0.0],[0.0],[0.0])) # wheel angle, wheel angular velocity, body angle, body angular velocity
        self.GUI_time = 0.0
        self.disturbance_timer = self.total_time / 2
        self.disturbance_switch = False

        urdf_file = os.path.join(os.path.dirname(__file__), "..", "urdf", "robot_model.urdf")
        urdf_file = os.path.abspath(urdf_file)
        
        physics_client = pyb.connect(pyb.GUI) 
        pyb.setTimeStep(self.loop_time) 
        pyb.setRealTimeSimulation(0)
        pyb.setAdditionalSearchPath(pybullet_data.getDataPath()) 
        pyb.setGravity(0,0,-9.81)
        pyb.setPhysicsEngineParameter(numSubSteps = self.sub_steps,
            numSolverIterations=self.solver_itterations) 

        self.plane_ID = pyb.loadURDF("plane.urdf") 
        pyb.resetDebugVisualizerCamera(cameraDistance=0.6,
                                     cameraYaw=90,
                                     cameraPitch=0,
                                     cameraTargetPosition=[0, 0, 0.1])
        self.robot_ID: int = pyb.loadURDF(urdf_file, flags=pyb.URDF_USE_IMPLICIT_CYLINDER)

        self.wheel_mass = pyb.getDynamicsInfo(self.robot_ID, 1)[0]
        self.wheel_radius = (pyb.getCollisionShapeData(self.robot_ID, 1)[0])[3][1]
        self.body_mass = pyb.getDynamicsInfo(self.robot_ID, 0)[0]
        self.wheel_MoI_x = 0.5 * self.wheel_mass * (self.wheel_radius ** 2)

        starting_offset_z = self.wheel_radius + self.wheel_base_offset * math.cos(abs(math.radians(self.start_angle)))
        starting_offset_y = -self.wheel_base_offset * math.sin(abs(math.radians(self.start_angle)))
        self.starting_position = [0.0, self.start_position + starting_offset_y, starting_offset_z]
        rotation_euler = Rotation.from_euler('xyz', [self.start_angle, 0, 0], degrees=True)
        rotation_quaternion = rotation_euler.as_quat()
        self.starting_orientation = rotation_quaternion
        pyb.resetBasePositionAndOrientation(self.robot_ID, self.starting_position, self.starting_orientation)

        self.set_gearbox_friction(1, actuator_friction_torque)
        self.set_gearbox_friction(2, actuator_friction_torque)


    def set_gearbox_friction(self, wheel_ID, actuator_friction_torque):
        pyb.changeDynamics(self.robot_ID, wheel_ID,
            lateralFriction=self.wheel_static_CoF,
            spinningFriction=self.wheel_static_CoF,
            rollingFriction=self.wheel_rolling_CoF)
        pyb.setJointMotorControl2(self.robot_ID, wheel_ID, pyb.VELOCITY_CONTROL, targetVelocity=0, force=actuator_friction_torque) # motor friction


    def save_state(self):
        jointA_pos, jointA_vel = pyb.getJointState(self.robot_ID, 1)[0:2]
        jointB_pos, jointB_vel = pyb.getJointState(self.robot_ID, 2)[0:2]
        self.state[0,0] = (jointA_pos + jointB_pos) / 2 
        self.state[1,0] = (jointA_vel + jointB_vel) / 2

        body_state = pyb.getLinkState(self.robot_ID, 0, computeLinkVelocity=True)
        rotation_quaternion = Rotation.from_quat(body_state[5])
        self.state[2, 0] = rotation_quaternion.as_euler('xyz', degrees=False)[0] 
        self.state[3,0] = body_state[7][0]


    def create_distrubance(self, loops):
        time_passed = loops * self.loop_time
        if ((time_passed - self.disturbance_timer) >= 0) and (self.disturbance_switch == True):
            if ((time_passed - self.disturbance_period - self.disturbance_timer) > 0):
                self.disturbance = 0
                self.disturbance_switch = False
            else:
                self.disturbance = np.random.uniform(-self.max_disturbance, self.max_disturbance)
            pyb.applyExternalForce(
                objectUniqueId=self.robot_ID,
                linkIndex=0,                    
                forceObj=[0, self.disturbance, 0],              
                posObj=[0, 0, 0.1],                 
                flags=pyb.LINK_FRAME)
            
        
    def set_actuator_torque(self, actuator_torque, actuator_ID):
        pyb.setJointMotorControl2(
            bodyUniqueId=self.robot_ID,
            jointIndex=actuator_ID, 
            controlMode=pyb.TORQUE_CONTROL, 
            force=actuator_torque)


    def correct_GUI_time(self):
        self.elapsed_time = time.time() - self.GUI_time
        self.sleep_time = self.loop_time * self.slow_motion_factor - self.elapsed_time
        if self.sleep_time > 0:
            time.sleep(self.sleep_time)
        self.GUI_time = time.time()


    def step(self):
        pyb.stepSimulation()


    def stop(self):
        pyb.disconnect()

