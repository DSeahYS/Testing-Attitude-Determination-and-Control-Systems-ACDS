import numpy as np
from skyfield.api import load, EarthSatellite, Topos
from geomag import geomag
from datetime import datetime, timezone
import utils.constants as const

class Environment:
    def __init__(self, tle_line1, tle_line2):
        self.ts = load.timescale()
        self.satellite = EarthSatellite(tle_line1, tle_line2, 'CubeSat', self.ts)
        self.earth = self.satellite.at(self.ts.now()).subpoint()

    def get_magnetic_field(self, position_eci, time):
        # Convert ECI to geodetic
        subpoint = self.satellite.at(time).subpoint()
        lat, lon, alt = subpoint.latitude.degrees, subpoint.longitude.degrees, subpoint.elevation.m / 1000  # km

        # Use geomag to get IGRF magnetic field
        mag = geomag.GeoMag(lat, lon, alt, time.utc_datetime().year, model=const.IGRF_MODEL)
        B_ned = np.array([mag.bx, mag.by, mag.bz])  # nT

        # Convert NED to ECI (simplified, assuming no rotation for now)
        # For full accuracy, need rotation matrix from NED to ECI
        B_eci = B_ned  # Placeholder

        return B_eci

    def get_orbital_elements(self, time):
        # Get position and velocity in ECI
        pos_vel = self.satellite.at(time)
        position = pos_vel.position.m
        velocity = pos_vel.velocity.m_per_s

        return position, velocity

    def quaternion_to_rotation_matrix(self, q):
        q0, q1, q2, q3 = q
        return np.array([
            [1 - 2*q2**2 - 2*q3**2, 2*q1*q2 - 2*q0*q3, 2*q1*q3 + 2*q0*q2],
            [2*q1*q2 + 2*q0*q3, 1 - 2*q1**2 - 2*q3**2, 2*q2*q3 - 2*q0*q1],
            [2*q1*q3 - 2*q0*q2, 2*q2*q3 + 2*q0*q1, 1 - 2*q1**2 - 2*q2**2]
        ])

    def get_disturbances(self, time, q):
        # Get position and velocity
        pos_vel = self.satellite.at(time)
        position = pos_vel.position.m  # ECI position
        velocity = pos_vel.velocity.m_per_s

        # Rotation matrix from ECI to body frame
        R_eci_to_body = self.quaternion_to_rotation_matrix(q)

        # Gravity gradient torque
        gravity_gradient_torque = self.compute_gravity_gradient_torque(position, R_eci_to_body)

        # Solar radiation pressure torque
        srp_torque = self.compute_srp_torque(time, R_eci_to_body)

        # Aerodynamic drag torque
        aerodynamic_torque = self.compute_aerodynamic_torque(velocity, R_eci_to_body)

        # Ephemeris for sun and moon positions (for future use)
        sun = load('de421.bsp')['sun']
        moon = load('de421.bsp')['moon']
        earth = load('de421.bsp')['earth']

        sun_pos = earth.at(time).observe(sun).position.m
        moon_pos = earth.at(time).observe(moon).position.m

        return {
            'gravity_gradient_torque': gravity_gradient_torque,
            'srp_torque': srp_torque,
            'aerodynamic_torque': aerodynamic_torque,
            'sun_position': sun_pos,
            'moon_position': moon_pos
        }

    def compute_gravity_gradient_torque(self, position, R_eci_to_body):
        r = np.linalg.norm(position)
        if r == 0:
            return np.zeros(3)

        # Nadir vector in ECI
        n_eci = -position / r

        # Transform to body frame
        n_body = R_eci_to_body @ n_eci

        # Inertia in body frame (assuming principal axes)
        I = np.array(const.CUBESAT_INERTIA)

        # Torque = 3 * mu / r^3 * (I @ n_body) × n_body
        mu = const.EARTH_MU
        factor = 3 * mu / r**3
        torque = factor * np.cross(I @ n_body, n_body)

        return torque

    def compute_srp_torque(self, time, R_eci_to_body):
        # Solar radiation pressure force
        # Simplified: assume CubeSat has solar panels, force on +X face
        # SRP pressure ~ 4.5e-6 Pa at 1 AU
        P_srp = 4.5e-6  # Pa
        A = const.CUBESAT_DIMENSIONS[0] * const.CUBESAT_DIMENSIONS[1]  # area of one face
        F_srp = P_srp * A  # force magnitude

        # Sun vector in ECI
        sun = load('de421.bsp')['sun']
        earth = load('de421.bsp')['earth']
        sun_pos = earth.at(time).observe(sun).position.m
        sun_eci = sun_pos / np.linalg.norm(sun_pos)

        # Transform to body frame
        sun_body = R_eci_to_body @ sun_eci

        # Assume force on +X face if sun is in +X direction
        # Torque = r × F, r = [0.05, 0, 0] (half dimension)
        r = np.array([const.CUBESAT_DIMENSIONS[0]/2, 0, 0])
        F = F_srp * sun_body[0] * np.array([1, 0, 0]) if sun_body[0] > 0 else np.zeros(3)
        torque = np.cross(r, F)

        return torque

    def compute_aerodynamic_torque(self, velocity, R_eci_to_body):
        # Simplified aerodynamic drag
        # Drag force proportional to v^2, torque due to offset CoM or attitude
        v_mag = np.linalg.norm(velocity)
        if v_mag == 0:
            return np.zeros(3)

        # Atmospheric density (simplified, constant)
        rho = 1e-12  # kg/m^3 at 400km
        Cd = 2.2  # drag coefficient
        A = const.CUBESAT_DIMENSIONS[0] * const.CUBESAT_DIMENSIONS[1]
        F_drag = 0.5 * rho * v_mag**2 * Cd * A

        # Velocity in body frame
        vel_body = R_eci_to_body @ velocity / v_mag

        # Assume drag on -velocity direction, torque if CoM offset
        # Simplified: small torque proportional to attitude
        torque = 1e-7 * vel_body  # small disturbance

        return torque