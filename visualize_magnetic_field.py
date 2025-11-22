"""
Pedagogical Visualization: Tilted Dipole Magnetic Field Model

Creates a 2D meridional slice showing the Earth's tilted dipole
magnetic field lines. This visualization helps understand why
the 11.5° tilt is crucial for B-Dot control.

As explained in the 6DOF research document Section 4.2:
Without the tilt, an equatorial satellite would see a constant
B-field vector and dB/dt would be zero, causing the B-Dot
controller to fail.
"""

import numpy as np
import matplotlib.pyplot as plt

# Constants
Re = 6371.0  # Earth Radius (km)
B0 = 3.12e-5  # Equatorial Field (Tesla)
tilt_deg = 11.5
tilt_rad = np.deg2rad(tilt_deg)

# Create grid in the Meridional Plane (X-Z plane, where Y=0)
# Using Y and Z for matplotlib convenience (Y horizontal, Z vertical)
y_range = np.linspace(-20000, 20000, 40)
z_range = np.linspace(-20000, 20000, 40)
Y, Z = np.meshgrid(y_range, z_range)
X = np.zeros_like(Y)  # Plane Y=0 (meridional slice)

# Dipole Moment Vector (Tilted)
mx = np.sin(tilt_rad)
mz = np.cos(tilt_rad)
m_hat = np.array([mx, 0, mz])

# Magnetic Field Calculation (Vectorized)
# Create r vector at each grid point
R_vec = np.stack((Y, X, Z), axis=-1)  # Using Y as "X-axis" for 2D plot ease
r = np.linalg.norm(R_vec, axis=-1)

# Mask inside Earth
mask = r > Re

# Dot product (m · r) at each point
dot_mr = Y*mx + Z*mz

# B-Field Components (Dipole Formula)
# B ~ (3(m·r)r - r²·m) / r^5
# We ignore B0 scaling for shape visualization
By = mask * (3 * dot_mr * Y - (r**2) * mx) / (r**5 + 1e-6)  # Add epsilon to avoid div by zero
Bz = mask * (3 * dot_mr * Z - (r**2) * mz) / (r**5 + 1e-6)

# Plotting
fig, ax = plt.subplots(figsize=(10, 10))

# Earth
circle = plt.Circle((0, 0), Re, color='#1e90ff', alpha=0.4, label='Earth', zorder=10)
ax.add_artist(circle)

# Magnetic field streamlines
# Color by log of field magnitude for better visualization
field_magnitude = np.sqrt(By**2 + Bz**2)
strm = ax.streamplot(
    Y, Z, By, Bz,
    color=np.log10(field_magnitude + 1e-20),
    cmap='plasma',
    density=1.8,
    linewidth=1.2,
    arrowsize=1.5
)

# Add colorbar
cbar = plt.colorbar(strm.lines, ax=ax, label='log₁₀(|B|)', pad=0.02)

# Dipole Axis Line (show the tilt)
axis_length = 18000
ax.plot(
    [0, axis_length * mx],
    [0, axis_length * mz],
    'r--',
    linewidth=3,
    label=f'Magnetic Dipole Axis ({tilt_deg}° Tilt)',
    zorder=15
)

# Geographic North (rotation axis) for comparison
ax.plot(
    [0, 0],
    [0, axis_length],
    'g--',
    linewidth=2,
    label='Geographic North (Rotation Axis)',
    alpha=0.7,
    zorder=15
)

# Annotations
ax.annotate(
    f'{tilt_deg}°',
    xy=(axis_length * mx * 0.3, axis_length * mz * 0.3),
    fontsize=14,
    color='red',
    weight='bold'
)

# Sample satellite orbit (equatorial, 400km altitude)
orbit_radius = Re + 400
theta = np.linspace(0, 2*np.pi, 100)
orbit_x = orbit_radius * np.cos(theta)
orbit_y = orbit_radius * np.sin(theta)
ax.plot(orbit_x, orbit_y, 'cyan', linewidth=2, label='Sample Orbit (400km, Equatorial)', alpha=0.8)

# Formatting
ax.set_aspect('equal')
ax.set_title(
    'Tilted Dipole Magnetic Field Model\n(Meridional Slice, Y=0 Plane)\n\n'
    'The 11.5° tilt creates time-varying dB/dt as Earth rotates,\n'
    'which is essential for B-Dot detumbling control',
    fontsize=13,
    weight='bold'
)
ax.set_xlabel('Equatorial Distance (km)', fontsize=12)
ax.set_ylabel('Polar Distance (km)', fontsize=12)
ax.legend(loc='upper right', fontsize=10)
ax.grid(True, alpha=0.3, linestyle=':', linewidth=0.8)
ax.set_xlim(-20000, 20000)
ax.set_ylim(-20000, 20000)

# Add text box explaining the physics
textstr = (
    'Physics Note:\n'
    '• Without tilt: Equatorial satellite sees constant B\n'
    '• With tilt: B rotates → dB/dt ≠ 0\n'
    '• B-Dot law: m = -k·dB/dt\n'
    '• Torque: τ = m × B (dissipates energy)'
)
props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
ax.text(
    0.02, 0.98,
    textstr,
    transform=ax.transAxes,
    fontsize=10,
    verticalalignment='top',
    bbox=props,
    family='monospace'
)

plt.tight_layout()

# Save figure
output_path = 'C:/VSCode Folder/ADCS Simulator/magnetic_dipole_visualization.png'
plt.savefig(output_path, dpi=200, bbox_inches='tight')
print(f"[✓] Magnetic field visualization saved to: {output_path}")

plt.show()
