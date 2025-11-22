import numpy as np
from scipy.integrate import ode
import utils.constants as const

class SatellitePhysics:
    def __init__(self, initial_attitude, initial_angular_velocity, environment):
        self.attitude = np.array(initial_attitude)  # quaternion [q0, q1, q2, q3]
        self.angular_velocity = np.array(initial_angular_velocity)  # rad/s
        self.inertia = np.array(const.CUBESAT_INERTIA)
        self.inertia_inv = np.linalg.inv(self.inertia)
        self.mode = 'detumbling'  # or 'pointing'
        self.environment = environment

    def quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        return np.array([
            [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
            [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
            [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]
        ])

    def dynamics(self, t, state):
        q = state[:4]
        omega = state[4:7]

        # Kinematics: quaternion derivative
        dq_dt = 0.5 * np.array([
            -q[1]*omega[0] - q[2]*omega[1] - q[3]*omega[2],
            q[0]*omega[0] + q[2]*omega[2] - q[3]*omega[1],
            q[0]*omega[1] - q[1]*omega[2] + q[3]*omega[0],
            q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0]
        ])

        # Torque: control + disturbances
        torque = self.compute_control_torque(omega) + self.compute_disturbance_torque(t, q, omega)

        # Dynamics: Euler's equations
        domega_dt = self.inertia_inv @ (torque - np.cross(omega, self.inertia @ omega))

        return np.concatenate([dq_dt, domega_dt])

    def compute_control_torque(self, omega):
        if self.mode == 'detumbling':
            # B-dot controller
            # Simplified: torque proportional to -omega
            k = 1e-3  # gain
            return -k * omega
        elif self.mode == 'pointing':
            # PD controller to point to nadir
            target_omega = np.array([0, 0, 0])
            kp = 1e-2
            kd = 1e-1
            return -kp * (omega - target_omega) - kd * omega
        return np.zeros(3)

    def compute_disturbance_torque(self, t, q, omega):
        # Get current time
        time = self.environment.ts.now() + t  # assuming t is relative

        # Get disturbances from environment
        disturbances = self.environment.get_disturbances(time, q)

        # Sum the torques
        total_torque = (
            disturbances['gravity_gradient_torque'] +
            disturbances['srp_torque'] +
            disturbances['aerodynamic_torque']
        )

        return total_torque

    def step(self, dt):
        # Integrate dynamics
        solver = ode(self.dynamics).set_integrator('dopri5')
        state = np.concatenate([self.attitude, self.angular_velocity])
        solver.set_initial_value(state, 0)
        solver.integrate(dt)
        new_state = solver.y
        self.attitude = new_state[:4] / np.linalg.norm(new_state[:4])  # normalize
        self.angular_velocity = new_state[4:7]

        # Autonomous handoff: if angular velocity low, switch to pointing
        if self.mode == 'detumbling' and np.linalg.norm(self.angular_velocity) < 0.01:
            self.mode = 'pointing'

    def get_state(self):
        return self.attitude, self.angular_velocity, self.mode