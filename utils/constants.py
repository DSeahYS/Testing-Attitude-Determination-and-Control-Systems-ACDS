# CubeSat parameters
CUBESAT_MASS = 1.33  # kg (1U CubeSat)
CUBESAT_DIMENSIONS = (0.1, 0.1, 0.1)  # m (length, width, height)
CUBESAT_INERTIA = [
    [0.0042, 0, 0],  # Ixx, Ixy, Ixz
    [0, 0.0042, 0],  # Iyx, Iyy, Iyz
    [0, 0, 0.0042]   # Izx, Izy, Izz
]  # kg*m^2

# Orbital parameters (example for ISS orbit)
ORBIT_ALTITUDE = 408000  # m
ORBIT_INCLINATION = 51.6  # degrees
ORBIT_ECCENTRICITY = 0.0001

# Earth parameters
EARTH_RADIUS = 6371000  # m
EARTH_MU = 3.986004418e14  # m^3/s^2

# Magnetic field model
IGRF_MODEL = 'IGRF13'

# Simulation parameters
SIMULATION_TIME_STEP = 0.1  # s
SIMULATION_DURATION = 3600  # s (1 hour)