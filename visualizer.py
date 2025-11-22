"""
VPython 3D Real-Time Visualization

Provides interactive 3D visualization of satellite orbit and attitude.
Shows orbital trajectory, satellite body with tumbling animation,
and real-time telemetry overlay.
"""

from vpython import (
    canvas, vector, sphere, box, arrow, curve, label, rate, color
)
import numpy as np

class Visualizer:
    """
    VPython-based 3D visualization for CubeSat ADCS simulation.
    
    Displays:
    - Earth sphere
    - Orbital trajectory
    - Satellite body (3U CubeSat)
    - Body frame axes (RGB = XYZ)
    - Real-time telemetry
    """
    
    def __init__(self, title="CubeSat ADCS Simulation - High Fidelity 6DOF"):
        """Initialize VPython scene and objects."""
        
        # Create scene
        self.scene = canvas(
            title=title,
            width=1200,
            height=800,
            center=vector(0, 0, 0),
            background=color.black
        )
        
        # Earth (scaled for visualization)
        self.earth_radius = 6371000.0  # meters
        self.scale = 1.0 / 1e6  # Scale: 1 VPython unit = 1000 km
        
        self.earth = sphere(
            pos=vector(0, 0, 0),
            radius=self.earth_radius * self.scale,
            color=color.blue,
            opacity=0.7,
            texture='https://i.imgur.com/rhFu6AI.jpg'  # Earth texture
        )
        
        # Satellite body (3U CubeSat: 10cm x 10cm x 30cm)
        self.sat_size = vector(0.1, 0.1, 0.3) * 20  # Scaled up for visibility
        self.satellite = box(
            pos=vector(0, 0, 0),
            size=self.sat_size,
            color=color.white
        )
        
        # Body frame axes (RGB = XYZ)
        arrow_length = 100
        self.axis_x = arrow(
            pos=vector(0, 0, 0),
            axis=vector(arrow_length, 0, 0),
            color=color.red,
            shaftwidth=5
        )
        self.axis_y = arrow(
            pos=vector(0, 0, 0),
            axis=vector(0, arrow_length, 0),
            color=color.green,
            shaftwidth=5
        )
        self.axis_z = arrow(
            pos=vector(0, 0, 0),
            axis=vector(0, 0, arrow_length),
            color=color.blue,
            shaftwidth=5
        )
        
        # Orbital trajectory curve
        self.trajectory = curve(
            color=color.yellow,
            radius=2
        )
        
        # Telemetry label
        self.telemetry = label(
            pos=vector(0, self.earth_radius * self.scale * 2, 0),
            text='Initializing...',
            height=12,
            border=6,
            font='monospace',
            color=color.white,
            background=color.black,
            opacity=0.7
        )
        
        # History tracking
        self.trajectory_history = []
        self.max_trajectory_points = 500
        
    def update(self, position_eci, quaternion, telemetry_data=None):
        """
        Update visualization with new state.
        
        Args:
            position_eci (np.array): Satellite position [x, y, z] in meters (ECI)
            quaternion (np.array): Attitude quaternion [qw, qx, qy, qz]
            telemetry_data (dict): Optional telemetry info for overlay
        """
        # Convert position to VPython coordinates (scaled)
        pos_scaled = vector(
            position_eci[0] * self.scale,
            position_eci[1] * self.scale,
            position_eci[2] * self.scale
        )
        
        # Update satellite position
        self.satellite.pos = pos_scaled
        
        # Update satellite orientation using quaternion
        # Convert quaternion to axis-angle for VPython
        qw, qx, qy, qz = quaternion
        
        # The rotation matrix method (more reliable for VPython)
        R = self._quaternion_to_rotation_matrix(quaternion)
        
        # Update satellite axes
        # Body X-axis (Red)
        x_body = vector(R[0, 0], R[1, 0], R[2, 0])
        self.satellite.axis = x_body * self.sat_size.x
        
        # Body frame arrows
        arrow_length = 100
        self.axis_x.pos = pos_scaled
        self.axis_x.axis = x_body * arrow_length
        
        self.axis_y.pos = pos_scaled
        y_body = vector(R[0, 1], R[1, 1], R[2, 1])
        self.axis_y.axis = y_body * arrow_length
        
        self.axis_z.pos = pos_scaled
        z_body = vector(R[0, 2], R[1, 2], R[2, 2])
        self.axis_z.axis = z_body * arrow_length
        
        # Update Up direction for box (Y-axis in VPython)
        self.satellite.up = y_body
        
        # Update trajectory
        self.trajectory.append(pos_scaled)
        self.trajectory_history.append(pos_scaled)
        
        # Limit trajectory length
        if len(self.trajectory_history) > self.max_trajectory_points:
            self.trajectory_history.pop(0)
            # Rebuild curve
            self.trajectory.clear()
            for pt in self.trajectory_history:
                self.trajectory.append(pt)
        
        # Update telemetry
        if telemetry_data:
            self._update_telemetry(telemetry_data)
        
        # Auto-adjust camera to follow satellite
        if len(self.trajectory_history) > 10:
            self.scene.center = pos_scaled
            
    def _quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion to rotation matrix (DCM).
        
        Args:
            q (np.array): Quaternion [qw, qx, qy, qz]
            
        Returns:
            np.array: 3x3 rotation matrix
        """
        qw, qx, qy, qz = q
        
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
            [2*qx*qy + 2*qw*qz, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qw*qx],
            [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        return R
    
    def _update_telemetry(self, data):
        """
        Update telemetry overlay.
        
        Args:
            data (dict): Dictionary with keys like 't', 'omega', 'mode', etc.
        """
        text_lines = []
        
        if 't' in data:
            text_lines.append(f"T+{data['t']:06.1f}s")
        
        if 'omega_mag' in data:
            text_lines.append(f"ω: {data['omega_mag']:.4f} rad/s")
        
        if 'torque_mag' in data:
            text_lines.append(f"τ: {data['torque_mag']:.2e} N·m")
        
        if 'altitude' in data:
            text_lines.append(f"Alt: {data['altitude']:.1f} km")
        
        if 'B_mag' in data:
            text_lines.append(f"B: {data['B_mag']*1e6:.1f} µT")
        
        self.telemetry.text = '\n'.join(text_lines)
    
    def add_reference_orbit(self, positions):
        """
        Add a reference orbit curve (e.g., from SGP4 at t=0).
        
        Args:
            positions (list): List of position vectors [x, y, z] in meters
        """
        ref_curve = curve(color=color.cyan, radius=1, opacity=0.3)
        for pos in positions:
            pos_scaled = vector(
                pos[0] * self.scale,
                pos[1] * self.scale,
                pos[2] * self.scale
            )
            ref_curve.append(pos_scaled)
