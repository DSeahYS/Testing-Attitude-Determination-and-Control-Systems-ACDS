# ADCS Simulator

A Python-based simulator for Attitude Determination and Control System (ADCS) of a CubeSat.

## Features

- Satellite dynamics simulation with quaternion kinematics and Euler's equations.
- Environment modeling with geomagnetic field (IGRF via geomag) and orbital propagation (SGP4 via skyfield).
- Disturbance torques including gravity gradient and solar pressure.
- Control modes: Detumbling (B-dot) and Pointing (PD controller).
- Autonomous handoff from detumbling to pointing when angular velocity is low.
- GUI dashboard for visualization.

## Installation

1. Install dependencies:
   ```
   pip install -r requirements.txt
   ```

2. Run the simulator:
   ```
   python main.py
   ```

## Structure

- `main.py`: Entry point.
- `physics/dynamics.py`: SatellitePhysics class for dynamics and control.
- `physics/environment.py`: Environment class for orbital and magnetic field data.
- `gui/dashboard.py`: ADCS_Dashboard class for GUI.
- `utils/constants.py`: CubeSat parameters.
- `requirements.txt`: Dependencies.

## Usage

The simulator starts in detumbling mode. Click "Update" to step the simulation. It will automatically switch to pointing mode when stabilized.