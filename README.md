# High-Fidelity 6DOF CubeSat ADCS Simulator

A comprehensive Python-based simulator for Attitude Determination and Control System (ADCS) of CubeSats, featuring multiple simulation interfaces and high-fidelity physics models.

![ADCS Simulator](https://github.com/user-attachments/assets/a8ce415d-a6c2-4e7b-9181-92be3d825e75)

## Features

### Physics Models
- **6DOF Rigid Body Dynamics**: Full quaternion kinematics and Euler's equations
- **Orbital Propagation**: SGP4 propagator via Skyfield library
- **Magnetic Field Modeling**: Tilted dipole model (11.5° tilt) with IGRF data
- **Disturbance Torques**: Gravity gradient, solar pressure, and magnetic effects
- **RK4 Integration**: High-precision numerical integration

### Control Algorithms
- **B-Dot Detumbling**: Passive magnetic detumbling with adaptive gain
- **PD Pointing Control**: Proportional-derivative attitude control
- **Autonomous Mode Switching**: Automatic transition from detumbling to pointing

### Simulation Interfaces
- **VPython 3D Visualization**: Real-time orbital and attitude visualization
- **Web Dashboard**: Interactive HTML5 canvas-based demos
- **GUI Dashboard**: Tkinter-based control interface
- **Terminal Simulation**: Command-line high-fidelity simulation
- **Testing Suite**: Comprehensive physics validation

### Research & Documentation
- **6DOF Research Paper**: Detailed mathematical formulation
- **Model-in-the-Loop (MIL) Verification**: Algorithm validation framework
- **Performance Metrics**: Energy dissipation, convergence analysis

## Installation

### Prerequisites
- Python 3.8+
- Modern web browser (for web interface)

### Dependencies
```bash
pip install -r requirements.txt
```

Required packages:
- `numpy` - Numerical computations
- `scipy` - Scientific computing
- `skyfield` - Orbital propagation
- `geomag` - Geomagnetic field calculations
- `matplotlib` - Plotting and visualization
- `vpython` - 3D visualization

## Usage

### High-Fidelity Terminal Simulation
Run a complete 3-orbit detumbling simulation with real-time telemetry:
```bash
python run_mission_hires.py
```

### Interactive Web Demo
Open `index.html` in your browser for interactive visualizations:
- Magnetic field animation
- B-Dot detumbling simulation
- Orbital mechanics demo

### GUI Dashboard
Launch the full control interface:
```bash
python main.py
```

### VPython 3D Visualization
Real-time 3D orbital and attitude visualization (requires VPython):
```bash
python run_mission_hires.py
```

### Testing & Validation
Run physics validation tests:
```bash
python test_physics.py
```

### Magnetic Field Visualization
Generate static magnetic field plots:
```bash
python visualize_magnetic_field.py
```

## Project Structure

```
ADCS Simulator/
├── main.py                    # GUI dashboard entry point
├── run_mission_hires.py       # High-fidelity terminal simulation
├── run_terminal_demo.py       # Simplified terminal demo
├── index.html                 # Interactive web dashboard
├── lab.html                   # Additional web experiments
├── visualizer.py              # VPython 3D visualization class
├── test_physics.py            # Physics validation tests
├── visualize_magnetic_field.py # Magnetic field plotting
├── requirements.txt           # Python dependencies
├── de421.bsp                  # JPL ephemeris data
├── simulation_output.txt      # Simulation logs
├── README.md                  # This file
├── Research Papers/
│   └── 6DOF.md               # Mathematical formulation
├── physics/
│   ├── satellite_physics.py  # Rigid body dynamics
│   ├── orbit.py              # SGP4 orbital propagation
│   ├── magnetic_model.py     # Tilted dipole field model
│   ├── bdot_controller.py    # B-Dot control algorithm
│   ├── solver.py             # RK4 numerical integration
│   ├── dynamics.py           # Legacy dynamics (GUI)
│   ├── environment.py        # Legacy environment (GUI)
│   └── __pycache__/          # Python bytecode
├── gui/
│   └── dashboard.py          # Tkinter GUI interface
└── utils/
    └── constants.py          # CubeSat parameters
```

## Configuration

### CubeSat Parameters (utils/constants.py)
- **Mass**: 4 kg (3U CubeSat)
- **Dimensions**: 10cm × 10cm × 30cm
- **Inertia Matrix**: Principal moments of inertia
- **Magnetic Properties**: Residual dipole moments

### Simulation Parameters
- **Time Step**: 50ms physics, 100ms control
- **B-Dot Gain**: 50,000 (adaptive)
- **Initial Conditions**: Configurable tumble rates
- **Orbital Elements**: ISS-like orbit (400km, 51.6°)

## Research Papers

### 6DOF Mathematical Formulation
Located in `Research Papers/6DOF.md`

Covers:
- Rigid body dynamics derivation
- Quaternion kinematics
- Control algorithm design
- Model validation methodology
- Performance analysis

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

This project is open-source. See LICENSE file for details.

## Acknowledgments

- Based on research from the 18th International Space Challenge
- Uses NASA SGP4 orbital propagator
- Magnetic field calculations via IGRF model
- VPython for 3D visualization
