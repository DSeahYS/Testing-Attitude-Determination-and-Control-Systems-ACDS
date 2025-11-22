"""
High-Fidelity 6DOF CubeSat ADCS Simulation

Main simulation loop integrating all physics modules:
- SGP4 Orbital Propagator
- Tilted Dipole Magnetic Model
- RK4 Rigid Body Dynamics
- True B-Dot Controller
- VPython 3D Visualization

This implementation follows the 6DOF research document architecture
for Model-in-the-Loop (MIL) verification of detumbling algorithms.
"""

import numpy as np
from vpython import rate

# Import High-Fidelity Physics Modules
from physics.orbit import Orbit
from physics.magnetic_model import MagneticModel
from physics.satellite_physics import SatellitePhysics
from physics.solver import RK4Solver
from physics.bdot_controller import BDotController
from visualizer import Visualizer

# ============================================================================
# CONFIGURATION
# ============================================================================

# TLE for ISS-like orbit (51.6° inclination, ~400km altitude)
TLE_L1 = "1 25544U 98067A   23250.00000000  .00000000  00000-0  00000-0 0  9999"
TLE_L2 = "2 25544  51.6400  10.0000 0002000   0.0000 000.0000 15.50000000    01"

# 3U CubeSat Inertia (from 6DOF research Section 5.2)
# Moment of inertia in kg·m² for 3U (3-4 kg, 10cm x 10cm x 30cm)
J_3U = [
    [0.11, 0, 0],
    [0, 0.0815, 0],
    [0, 0, 0.0815]
]

# B-Dot Controller Gain (Section 7.4)
# Large value needed to convert micro-Tesla/second to meaningful torque
GAIN_BDOT = 50000.0

# Simulation Parameters
DT_PHYSICS = 0.05  # 20 Hz physics update
DT_CONTROL = 0.1   # 10 Hz control update
SIMULATION_DURATION = 3 * 90 * 60  # 3 orbits (~270 minutes)

# Initial Conditions
# High tumble rate to demonstrate detumbling
INITIAL_ATTITUDE = [1.0, 0.0, 0.0, 0.0]  # [qw, qx, qy, qz] - aligned with ECI
INITIAL_OMEGA = [0.2, -0.3, 0.1]  # rad/s - significant tumble

# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def quaternion_conjugate(q):
    """Return quaternion conjugate (inverse rotation)."""
    return np.array([q[0], -q[1], -q[2], -q[3]])

def rotate_vector_by_quaternion(q, v):
    """Rotate vector v by quaternion q."""
    q_vec = q[1:]
    q_scalar = q[0]
    t = 2 * np.cross(q_vec, v)
    v_prime = v + q_scalar * t + np.cross(q_vec, t)
    return v_prime

# ============================================================================
# MAIN SIMULATION
# ============================================================================

def run_mission():
    """
    Execute high-fidelity CubeSat ADCS simulation.
    
    Control Loop Architecture (20 Hz):
    1. Get position from orbit propagator → r_ECI
    2. Get magnetic field from environment → B_ECI
    3. Transform B-field to body frame → B_body (simulates magnetometer)
    4. Compute control dipole (B-Dot) → m_dipole
    5. Calculate torque → τ = m × B
    6. Integrate dynamics (RK4) → new state
    7. Visualize
    """
    print("=" * 70)
    print("HIGH-FIDELITY 6DOF CUBESAT ADCS SIMULATION")
    print("=" * 70)
    print("\n[INIT] Booting physics engine...\n")
    
    # ========================================================================
    # 1. Initialize Modules
    # ========================================================================
    
    orbit = Orbit(TLE_L1, TLE_L2)
    print(f"[✓] Orbit: SGP4 propagator ready")
    print(f"    Orbital period: {orbit.get_orbital_period()/60:.1f} minutes")
    
    mag_model = MagneticModel()
    print(f"[✓] Magnetic Model: Tilted dipole (11.5° tilt)")
    
    sat_physics = SatellitePhysics(J_3U)
    print(f"[✓] Dynamics: Rigid body engine")
    print(f"    Inertia: diag({J_3U[0][0]:.3f}, {J_3U[1][1]:.4f}, {J_3U[2][2]:.4f}) kg·m²")
    
    solver = RK4Solver(sat_physics)
    print(f"[✓] Solver: RK4 integrator (dt={DT_PHYSICS}s)")
    
    controller = BDotController(gain=GAIN_BDOT, dt_control=DT_CONTROL)
    print(f"[✓] Controller: B-Dot (K={GAIN_BDOT:.0f})")
    
    viz = Visualizer()
    print(f"[✓] Visualizer: VPython 3D")

    # ========================================================================
    # 2. Initial State
    # ========================================================================
    
    # State vector: [qw, qx, qy, qz, wx, wy, wz]
    state = np.array(INITIAL_ATTITUDE + INITIAL_OMEGA, dtype=np.float64)
    
    print(f"\n[INIT] Initial Conditions:")
    print(f"    Attitude: q = [{state[0]:.2f}, {state[1]:.2f}, {state[2]:.2f}, {state[3]:.2f}]")
    print(f"    Angular velocity: ω = [{state[4]:.2f}, {state[5]:.2f}, {state[6]:.2f}] rad/s")
    print(f"    ω magnitude: {np.linalg.norm(state[4:7]):.4f} rad/s")
    
    # ========================================================================
    # 3. Simulation Loop
    # ========================================================================
    
    print(f"\n[MISSION] Simulation Start")
    print(f"[MISSION] Duration: {SIMULATION_DURATION/60:.0f} minutes ({SIMULATION_DURATION/(90*60):.1f} orbits)")
    print("=" * 70)
    
    t = 0.0
    control_timer = 0.0
    telemetry_timer = 0.0
    
    # Torque starts at zero
    torque_body = np.zeros(3)
    
    while t < SIMULATION_DURATION:
        # VPython rate limiter (real-time playback)
        rate(20)
        
        # ====================================================================
        # A. ENVIRONMENT STEP
        # ====================================================================
        
        # 1. Get ECI Position (SGP4)
        r_eci, t_skyfield = orbit.get_position_ECI(t)
        altitude = (np.linalg.norm(r_eci) - mag_model.Re) / 1000.0  # km
        
        # 2. Get Magnetic Field in ECI (Tilted Dipole)
        b_eci = mag_model.get_field_ECI(r_eci, t)
        
        # ====================================================================
        # B. SENSOR STEP
        # ====================================================================
        
        # 3. Rotate B-field to Body Frame (Magnetometer Reading)
        # ECI -> Body requires quaternion conjugate (inverse rotation)
        q_curr = state[0:4]
        q_conj = quaternion_conjugate(q_curr)
        b_body = rotate_vector_by_quaternion(q_conj, b_eci)
        
        # ====================================================================
        # C. CONTROL STEP
        # ====================================================================
        
        # Only update control at control frequency (10 Hz)
        control_timer += DT_PHYSICS
        if control_timer >= DT_CONTROL:
            control_timer = 0.0
            
            # 4. Compute Dipole Command (B-Dot)
            m_dipole_body = controller.compute_dipole(b_body)
            
            # 5. Compute Torque (m × B)
            torque_body = np.cross(m_dipole_body, b_body)
        
        # ====================================================================
        # D. DYNAMICS STEP
        # ====================================================================
        
        # 6. Integrate (RK4)
        state = solver.step(t, state, DT_PHYSICS, torque_body)
        
        # ====================================================================
        # E. VISUALIZATION
        # ====================================================================
        
        # Prepare telemetry data
        omega_mag = np.linalg.norm(state[4:7])
        torque_mag = np.linalg.norm(torque_body)
        b_mag = np.linalg.norm(b_body)
        
        telemetry_data = {
            't': t,
            'omega_mag': omega_mag,
            'torque_mag': torque_mag,
            'altitude': altitude,
            'B_mag': b_mag
        }
        
        viz.update(r_eci, state[0:4], telemetry_data)
        
        # ====================================================================
        # F. TELEMETRY (Console Output)
        # ====================================================================
        
        telemetry_timer += DT_PHYSICS
        if telemetry_timer >= 2.0:  # Print every 2 seconds
            telemetry_timer = 0.0
            
            print(f"T+{t:06.1f}s | ω: {omega_mag:.4f} rad/s | τ: {torque_mag:. 2e} N·m | Alt: {altitude:.1f} km | B: {b_mag*1e6:.1f} µT")
            
            # Check for successful detumbling
            if omega_mag < 0.01:
                print("\n" + "=" * 70)
                print(f"[SUCCESS] Satellite detumbled at T+{t:.1f}s")
                print(f"[SUCCESS] Final ω = {omega_mag:.6f} rad/s")
                print("=" * 70)
                # Continue simulation to show stable state
        
        # ====================================================================
        # G. TIME STEP
        # ====================================================================
        
        t += DT_PHYSICS
    
    print("\n" + "=" * 70)
    print("[MISSION] Simulation Complete")
    print("=" * 70)

if __name__ == "__main__":
    """Entry point for high-fidelity simulation."""
    try:
        run_mission()
    except KeyboardInterrupt:
        print("\n\n[INFO] Simulation interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Simulation failed: {e}")
        import traceback
        traceback.print_exc()
