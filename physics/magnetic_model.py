"""
Tilted Dipole Magnetic Field Model

Implements Section 4 of the 6DOF research document.
Provides ~90% accuracy with 10x faster computation than IGRF.

Mathematical basis:
    B(r) = (μ_e / r^5) * [3(m·r)r - r²m]
    
Where the dipole axis m rotates with Earth at 11.5° tilt.
"""

import numpy as np

class MagneticModel:
    """
    Implements the Tilted Dipole Model for Earth's Magnetic Field.
    Provides the magnetic field vector in the ECI (Inertial) frame.
    """
    
    def __init__(self):
        """Initialize magnetic field model parameters."""
        # Physical Constants
        self.Re = 6371000.0  # Earth Mean Radius (m)
        self.B0 = 3.12e-5    # Equatorial Field Strength (Tesla)
        self.mu_e = self.B0 * (self.Re ** 3)  # Earth's Magnetic Moment
        
        # Dipole Parameters
        self.dipole_tilt = np.deg2rad(11.5)  # Tilt relative to rotation axis
        self.w_earth = 7.2921159e-5  # Earth rotation rate (rad/s)
        
    def get_field_ECI(self, r_eci, t_seconds):
        """
        Calculate B-field in ECI frame at position r_eci and time t.
        
        The critical feature is that the dipole axis rotates with Earth,
        creating the time-varying dB/dt signal that the B-Dot controller
        exploits for detumbling.
        
        Args:
            r_eci (np.array): Position vector [x, y, z] in meters (ECI frame)
            t_seconds (float): Simulation time in seconds
            
        Returns:
            np.array: Magnetic field vector [Bx, By, Bz] in Tesla (ECI frame)
        """
        r_norm = np.linalg.norm(r_eci)
        
        # Sanity check
        if r_norm < self.Re:
            raise ValueError(f"Satellite is underground! r={r_norm/1000:.1f} km < Re={self.Re/1000:.1f} km")
        
        # 1. Determine Dipole Axis Orientation in ECI
        # The dipole rotates with the Earth about the Z-axis
        alpha = self.w_earth * t_seconds
        
        # Dipole unit vector (m_hat)
        # Assumes dipole starts in X-Z plane at t=0 for simplicity
        mx = np.sin(self.dipole_tilt) * np.cos(alpha)
        my = np.sin(self.dipole_tilt) * np.sin(alpha)
        mz = np.cos(self.dipole_tilt)
        m_hat = np.array([mx, my, mz])
        
        # 2. Calculate Dipole Field Vector Formula
        # B = (mu_e / r^5) * (3(m·r)r - r² * m)
        
        dot_product = np.dot(m_hat, r_eci)
        
        term1 = 3 * dot_product * r_eci
        term2 = (r_norm**2) * m_hat
        
        B_vec = (self.mu_e / (r_norm**5)) * (term1 - term2)
        
        return B_vec
    
    def get_field_magnitude(self, r_eci):
        """
        Calculate approximate field magnitude (useful for visualization).
        
        Args:
            r_eci (np.array): Position vector in meters
            
        Returns:
            float: Approximate field magnitude in Tesla
        """
        r_norm = np.linalg.norm(r_eci)
        # Dipole field magnitude scales as 1/r³
        return self.B0 * (self.Re / r_norm) ** 3
