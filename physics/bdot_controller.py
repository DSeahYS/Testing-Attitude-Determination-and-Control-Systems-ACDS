"""
B-Dot Magnetic Detumbling Controller

Implements Section 7 of the 6DOF research document.
True derivative-based control law with actuator saturation.

Control Law: m = -k · dB/dt

The controller exploits the time-varying magnetic field to
dissipate rotational kinetic energy without targeting a
specific attitude (momentum management only).
"""

import numpy as np

class BDotController:
    """
    B-Dot Magnetic Detumbling Controller.
    
    Unlike simplified implementations that use -k·ω, this implements
    the true discrete derivative of the magnetic field vector in the
    body frame, matching real flight software architecture.
    """
    
    def __init__(self, gain, dt_control, max_dipole=0.15):
        """
        Initialize B-Dot controller.
        
        Args:
            gain (float): Control gain K (typically 10⁴ - 10⁶)
                Large gain needed to convert micro-Tesla changes to torque
            dt_control (float): Control loop timestep in seconds
            max_dipole (float): Magnetorquer saturation limit in A·m²
                Default 0.15 is typical for 1U CubeSat
        """
        self.k = gain
        self.dt = dt_control
        self.last_B_body = None
        self.max_dipole = max_dipole
        
    def compute_dipole(self, current_B_body):
        """
        Calculates magnetic dipole command based on B-field rate of change.
        
        Theory:
            dB/dt in body frame ≈ -ω × B (for tumbling satellite)
            Therefore, -k·dB/dt produces torque opposing rotation
            
        Args:
            current_B_body (np.array): Current magnetic field in body frame [T]
            
        Returns:
            np.array: Commanded magnetic dipole moment [A·m²]
        """
        # Handle first step (no derivative possible)
        if self.last_B_body is None:
            self.last_B_body = current_B_body.copy()
            return np.zeros(3)
            
        # 1. Discrete Derivative (backward difference)
        # dB/dt ≈ (B[k] - B[k-1]) / Δt
        b_dot = (current_B_body - self.last_B_body) / self.dt
        
        # 2. Control Law (Proportional to -dB/dt)
        # m = -K · dB/dt
        m_command = -self.k * b_dot
        
        # 3. Saturation (Actuator Limits)
        # Real magnetorquers have finite dipole strength
        # Clip each axis independently
        m_command = np.clip(m_command, -self.max_dipole, self.max_dipole)
        
        # Update memory for next iteration
        self.last_B_body = current_B_body.copy()
        
        return m_command
    
    def compute_torque(self, B_body):
        """
        Convenience method: Compute control torque directly.
        
        Combines dipole calculation and cross product τ = m × B.
        
        Args:
            B_body (np.array): Magnetic field in body frame [T]
            
        Returns:
            np.array: Control torque [N·m]
        """
        m_dipole = self.compute_dipole(B_body)
        torque = np.cross(m_dipole, B_body)
        return torque, m_dipole
    
    def reset(self):
        """Reset controller state (useful for simulation restart)."""
        self.last_B_body = None
        
    def get_commanded_dipole(self):
        """
        Get the last commanded dipole (for telemetry).
        
        Returns:
            np.array or None: Last dipole command
        """
        if self.last_B_body is None:
            return None
        # Recompute from stored state (approximate)
        return -self.k * (self.last_B_body / self.dt)
