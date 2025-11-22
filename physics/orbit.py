"""
SGP4-based Orbital Propagator with Coordinate Frame Management

This module implements Section 3 of the 6DOF research document.
Uses Skyfield for TLE parsing, SGP4 propagation, and automatic
TEME -> GCRS/J2000 coordinate conversion.
"""

import numpy as np
from skyfield.api import load, EarthSatellite

class Orbit:
    """
    Handles SGP4 propagation and Coordinate Frame Transformations.
    
    The orbit is decoupled from rotational dynamics as recommended
    for future SIL/HIL extensibility.
    """
    
    def __init__(self, tle_line1, tle_line2):
        """
        Initialize orbital propagator from TLE.
        
        Args:
            tle_line1 (str): First line of TLE
            tle_line2 (str): Second line of TLE
        """
        self.ts = load.timescale()
        self.sat = EarthSatellite(tle_line1, tle_line2, 'CubeSat', self.ts)
        self.epoch = self.sat.epoch
        
    def get_position_ECI(self, t_seconds_from_start):
        """
        Returns position in ECI (GCRS/J2000) frame.
        
        SGP4 internally uses TEME frame, but Skyfield automatically
        converts to GCRS when using the .position attribute.
        
        Args:
            t_seconds_from_start (float): Simulation time in seconds
            
        Returns:
            tuple: (position_meters, skyfield_time)
                - position_meters: np.array [x, y, z] in meters
                - skyfield_time: Skyfield Time object for field lookup
        """
        # Create Skyfield Time object
        # Note: We assume start time is the TLE epoch for simplicity
        t_current = self.ts.tt_jd(self.epoch.tt + t_seconds_from_start / 86400.0)
        
        # Propagate using SGP4 (outputs in TEME, converted to GCRS)
        geocentric = self.sat.at(t_current)
        
        # Skyfield outputs position in km, convert to meters (SI units)
        pos_km = geocentric.position.km
        pos_meters = pos_km * 1000.0
        
        return pos_meters, t_current
    
    def get_orbital_period(self):
        """
        Calculate approximate orbital period from TLE.
        
        Returns:
            float: Orbital period in seconds
        """
        # Mean motion is in revolutions per day
        mean_motion = self.sat.model.no_kozai  # radians per minute
        period_minutes = (2 * np.pi) / mean_motion
        return period_minutes * 60.0  # Convert to seconds
