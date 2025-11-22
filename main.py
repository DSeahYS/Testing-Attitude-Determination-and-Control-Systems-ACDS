from physics.environment import Environment
from physics.dynamics import SatellitePhysics
from gui.dashboard import ADCS_Dashboard
import numpy as np

# Example TLE for ISS (replace with CubeSat TLE)
tle_line1 = "1 25544U 98067A   23250.00000000  .00000000  00000-0  00000-0 0  9999"
tle_line2 = "2 25544  51.6400  10.0000 0002000   0.0000 000.0000 15.50000000    01"

def main():
    # Initialize environment
    env = Environment(tle_line1, tle_line2)

    # Initial conditions
    initial_attitude = [1, 0, 0, 0]  # quaternion
    initial_omega = [0.1, 0.1, 0.1]  # rad/s

    # Initialize satellite physics
    satellite = SatellitePhysics(initial_attitude, initial_omega, env)

    # Initialize dashboard
    dashboard = ADCS_Dashboard(satellite, env)

    # Run the dashboard
    dashboard.run()

if __name__ == "__main__":
    main()