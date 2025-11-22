Comprehensive Design, Mathematical Derivation, and Python Implementation of High-Fidelity 6-DOF CubeSat Attitude Control Simulations
1. Introduction: The Imperative of High-Fidelity Simulation in Nanosatellite Missions
The democratization of space access through the CubeSat standard has precipitated a paradigm shift in orbital mission design. However, the miniaturization of satellite hardware imposes severe constraints on the Attitude Determination and Control System (ADCS). Unlike large monolith satellites with redundant reaction wheels and star trackers, CubeSats often rely on magnetic actuation—specifically magnetorquers—for momentum management and detumbling. The efficacy of such control laws, particularly the B-Dot algorithm, is inextricably linked to the complex, non-linear dynamics of the spacecraft and its interaction with the geomagnetic environment. Consequently, the development of a high-fidelity "physics engine" is not merely a software exercise but a mission-critical requirement for validating flight software before deployment.

This report serves as a comprehensive technical reconstruction of the "missing" physics and mathematics modules required to operationalize a Python-based satellite simulation, such as the Advitiy-Control-Model referencing the Student-Satellite-IITB architecture. While many open-source repositories provide control logic, they frequently abstract away or omit the underlying rigid body dynamics and environmental modeling—the "Plant" in control theory terms. To enable a functional simulation, one must reconstruct the physics engine from first principles, integrating orbital mechanics, environmental vector fields, rigid body kinematics, and numerical integration schemes.

The analysis draws upon a wide array of technical documentation, from the implementation details of the SGP4 propagator in Python libraries like skyfield , to the theoretical underpinnings of magnetic detumbling , and the intricacies of quaternion-based numerical integration. The objective is to provide a blueprint for a Model-In-the-Loop (MIL) simulation environment that is physically rigorous, mathematically consistent, and computationally efficient.   

1.1 The Role of the Physics Engine in Closed-Loop Control
In the context of satellite GNC (Guidance, Navigation, and Control), the simulation architecture is bipartite: the Flight Software (FSW) and the Environmental/Dynamics Simulator (Sim). The FSW dictates what the satellite should do (e.g., "activate X-axis magnetorquer"), while the Sim determines what actually happens (e.g., "the satellite rotates 0.1 radians due to torque").

The "missing files" alluded to in the user query typically correspond to the Sim component, specifically:

The Environmental Model: The digital representation of the Earth's magnetic field, atmospheric density, and solar pressure.

The Dynamics Core: The differential equation solver that integrates Euler’s rotational equations of motion.

The Kinematic Propagator: The mechanism for tracking orientation (attitude) over time, invariably utilizing quaternions to avoid singularities.

Without these components, the B-Dot controller—which relies on the rate of change of the magnetic field vector—has no input to process and no physical system to act upon. The simulation must therefore bridge the gap between the discrete control logic and the continuous physical world.   

1.2 Simulation Architectures: MIL, SIL, and HIL
The development of the physics engine allows for various testing modalities. The primary focus here is Model-in-the-Loop (MIL), where the control algorithms (written in Python) interact directly with the physics model (also in Python). This is the most flexible stage, allowing for rapid iteration of gains and logic.   

However, the Python-based physics engine must be designed with future extensibility in mind. As the mission matures, the same physics backend can drive a Software-in-the-Loop (SIL) test, where the actual C/C++ flight code is wrapped and stimulated by the Python environment. Eventually, this extends to Hardware-in-the-Loop (HIL), where the Python simulation generates magnetic field vectors that are fed into physical magnetometers to trick the hardware into believing it is in orbit. The data suggests that separating the Physics and Environment classes is crucial for this scalability, allowing individual modules to be replaced by hardware interfaces without refactoring the entire simulation.   

2. Computational Astrodynamics and Coordinate Reference Systems
The foundation of any aerospace simulation is the rigorous definition of Coordinate Reference Systems (CRS). A simulation that fails to distinguish between the Inertial Frame (where Newton's laws apply) and the Body Frame (where sensors measure) will yield physically meaningless results. The "missing" physics files must explicitly handle the transformations between these frames at every time step.

2.1 The Earth-Centered Inertial (ECI) Frame
The Equations of Motion (EOM) for a satellite are valid only in an inertial frame. For Earth-orbiting satellites, the Earth-Centered Inertial (ECI) frame is the standard.

Origin: Geometric center of the Earth.

X-Axis: Points toward the Vernal Equinox (the First Point of Aries, γ).

Z-Axis: Aligned with the Earth's rotation axis (North Pole).

Y-Axis: Completes the right-handed orthogonal set ( 
Z

 × 
X

 ).

In Python astrodynamics libraries like skyfield or astropy, this is often referred to as the GCRS (Geocentric Celestial Reference System) or J2000 frame. The physics engine must track the satellite's position ( 
r

 ) and velocity ( 
v

 ) in this frame to propagate the orbit using SGP4.   

2.2 The Earth-Centered Earth-Fixed (ECEF) Frame
While the satellite moves in ECI, the environment (Earth) rotates beneath it. The magnetic field models (IGRF, WMM) are defined relative to the Earth's geography. This requires the Earth-Centered Earth-Fixed (ECEF) frame, also known as ITRS (International Terrestrial Reference System).   

Origin: Geometric center of the Earth.

X-Axis: Intersection of the Equator and the Prime Meridian (Greenwich).

Z-Axis: Aligned with the Earth's rotation axis (North Pole).

Y-Axis: 90 
∘
  East longitude in the equatorial plane.

The transformation from ECEF to ECI depends on the Earth's rotation angle (Greenwich Sidereal Time, GST) and effects like precession and nutation. The skyfield library handles these transformations automatically using high-precision IERS data, which is a significant advantage over writing manual rotation matrices.   

2.3 The Body Frame (BF)
The Body Frame is rigidly attached to the satellite structure. This is the frame of relevance for the ADCS.

Origin: Center of Mass (CoM) of the satellite.

Axes: Typically aligned with the geometric faces of the CubeSat. For a 1U CubeSat, these are the face normals.

Relevance:

Inertia Tensor (J): Constant in the Body Frame.

Sensors: Magnetometers measure the magnetic field vector projected onto the Body axes (B 
body
​
 ).

Actuators: Magnetorquers produce dipoles along Body axes (m 
body
​
 ).

The physics simulation's primary job is to track the orientation of the Body Frame relative to the ECI Frame. This orientation is represented by the attitude quaternion q 
ECI→Body
​
 .   

2.4 The NEC (North-East-Center) and NED Frames
For visualization and ground track analysis, the simulation often converts positions to Geodetic coordinates (Latitude, Longitude, Altitude). While not strictly required for the physics integration loop, these are essential for the "Mission Control" dashboard visualization mentioned in the query context.   

Table 1: Summary of Coordinate Systems Required for ADCS Simulation

Frame Acronym	Full Name	Fixed To	Use Case in Simulation	Python Implementation
ECI / GCRS	Earth-Centered Inertial	Stars (Vernal Equinox)	Integration of Equations of Motion (Newton's Laws)	skyfield.api.GCRS
ECEF / ITRS	Earth-Centered Earth-Fixed	Earth (Greenwich)	Magnetic Field Lookup (IGRF), Gravity Models	skyfield.api.ITRS
TEME	True Equator Mean Equinox	Epoch Date	SGP4 TLE Propagation (Specific ECI variant)	skyfield.sgp4lib.TEME
Body	Satellite Body	Satellite Structure	Sensors, Actuators, Inertia Tensor	Custom numpy arrays
2.5 Time Standards and SGP4
Astrodynamics requires precise timekeeping. The SGP4 propagator, which drives the satellite's orbital position based on Two-Line Elements (TLEs), utilizes the TEME (True Equator Mean Equinox) frame and requires time inputs in Julian Dates or specific epoch offsets. The Python skyfield library provides a Timescale object that manages the conversion between UTC (Coordinated Universal Time), TAI (International Atomic Time), and TT (Terrestrial Time). The "missing physics" module must instantiate a Timescale object at the start of the simulation to ensure that the orbital propagation remains synchronized with the Earth's rotation for accurate magnetic field lookup.   

3. Orbital Mechanics: The SGP4 Propagator Implementation
The first step in the physics loop is determining where the satellite is. While full numerical integration of the orbital position (Cowell's method) is possible, it is computationally expensive and unnecessary for ADCS testing. The industry standard for LEO (Low Earth Orbit) satellites is the SGP4 (Simplified General Perturbations 4) model, which propagates the satellite's position using TLE data.

3.1 SGP4 Theory and TLEs
SGP4 is a semi-analytical model that accounts for the Earth's oblateness (J 
2
​
  zonal harmonic) and atmospheric drag. It takes a TLE as input. A TLE (Two-Line Element) set encodes the Keplerian elements (Inclination, Eccentricity, etc.) and drag terms (B-Star) at a specific epoch.

Example TLE Structure : 1 25544U 98067A 21275.52954488 .00006207 00000-0 12137-3 0 9992 2 25544 51.6430 339.9236 0004788 238.9771 216.1209 15.48758826305272 The SGP4 algorithm, accessible via skyfield or the sgp4 Python library, parses these lines and outputs the position ( 
r

 ) and velocity ( 
v

 ) vectors in the TEME frame.   

3.2 The TEME to ECI Conversion Nuance
A critical implementation detail often missed is that SGP4 outputs in TEME, not standard J2000 ECI. While the difference is small (on the order of kilometers/meters), precise magnetic field modeling requires converting TEME to the simulation's standard inertial frame (usually J2000). Snippet  highlights the discrepancy in position vectors when this conversion is mishandled. The skyfield library handles this implicitly if the correct methods are called, converting satellite.at(t) into a GCRS (J2000) vector automatically.   

3.3 Constructing the Orbit Module
To resolve the missing files, we must define an Orbit class that encapsulates this complexity. This class acts as the driver for the translational physics.

Proposed Implementation Logic:

Initialization: Load TLE strings and generate a Skyfield EarthSatellite object.

Propagation: Acceptance of a simulation time t (seconds since start).

Conversion: Conversion of the simulation time to a Skyfield Time object.

Query: retrieval of the geometric position.

Output: Return of position and velocity in SI units (meters and m/s). Note that skyfield defaults to Au (Astronomical Units) or km/day, so explicit conversion to km and km_per_s (then to meters) is mandatory.   

Insight on Error Accumulation: Snippet  notes that SGP4 propagation error grows over time (km per day). For a detumbling simulation lasting a few orbits (approx. 90 minutes per orbit), this error is negligible regarding magnetic field sampling. The magnetic field changes slowly enough that a position error of a few kilometers results in a magnetic vector error well below the noise floor of standard CubeSat magnetometers.   

4. Environmental Modeling: The Geomagnetic Field
The B-Dot controller is purely magnetic. Therefore, the accuracy of the simulation depends almost entirely on the fidelity of the magnetic field model. The "missing environment" file must implement a mathematical model that takes an ECI position and time, and returns the magnetic field vector  
B

  in the ECI frame.

4.1 Modeling Approaches: IGRF vs. Tilted Dipole
There are two prevailing methods for simulating the geomagnetic field:

IGRF (International Geomagnetic Reference Field): This is the gold standard, utilizing a spherical harmonic expansion up to degree and order 13. It accounts for magnetic anomalies (like the South Atlantic Anomaly) and secular variation. However, calculating spherical harmonics up to degree 13 at every simulation step (e.g., 10 Hz) can be computationally intensive in pure Python.   

Tilted Dipole Model: This approximates the Earth's field as a perfect magnetic dipole, but tilted relative to the rotation axis. As noted in , the dipole accounts for ~90% of the field energy. For testing detumbling algorithms, the Tilted Dipole is often preferred due to its simplicity and speed, while still capturing the essential rotational dynamics of the field required for B-Dot to function.   

4.2 Mathematical Derivation of the Tilted Dipole
The magnetic field  
B

  of a dipole is given by the vector equation:

B

 ( 
r

 )= 
4π
μ 
0
​
 
​
  
r 
3
 
3( 
m

 ⋅ 
r
^
 ) 
r
^
 − 
m

 
​
 
Alternatively, using the Earth's magnetic moment magnitude M 
E
​
 ≈7.94×10 
22
  A⋅m 
2
  (often represented as B 
0
​
 R 
E
3
​
 ): $$ \vec{B}(\vec{r}) = \frac{B_0 R_E^3}{r^5} \left( 3(\vec{m}{unit} \cdot \vec{r})\vec{r} - r^2 \vec{m}{unit} \right) $$

Where:

B 
0
​
 ≈3.12×10 
−5
  Tesla (Equatorial field strength).

R 
E
​
 =6371 km (Earth Radius).

r

  is the position vector of the satellite from Earth's center.

m

  
unit
​
  is the unit vector of the dipole axis.   

The Critical Rotation: The Earth's magnetic dipole is tilted by approximately θ 
tilt
​
 ≈11.5 
∘
  relative to the rotation axis. Crucially, this dipole rotates with the Earth. In the ECI frame, the dipole vector  
m

  
unit
​
  is not constant; it traces a cone. $$ \vec{m}{ECI}(t) = \begin{bmatrix} \sin(\theta{tilt}) \cos(\omega_e t + \phi_0) \ \sin(\theta_{tilt}) \sin(\omega_e t + \phi_0) \ \cos(\theta_{tilt}) \end{bmatrix} $$ Where ω 
e
​
  is the Earth's rotation rate (≈7.292×10 
−5
  rad/s) and ϕ 
0
​
  is the initial longitude of the dipole. The "missing physics code" must implement this time-dependent rotation. If the dipole is modeled as static in ECI, the B-Dot controller (which relies on  
B
˙
 ) will see a significantly different derivative, potentially leading to false validation of the control law.   

4.3 Python Implementation Strategy (MagneticModel Class)
The reconstructed MagneticModel class should offer a method get_field_ECI(position, time).

Inputs: Position  
r

  (meters) in ECI, Time t (seconds).

Dipole Rotation: Calculate the current orientation of the dipole axis  
m

 (t) based on the Earth's rotation rate.

Vector Math: Compute the dot product  
m

 ⋅ 
r

  and apply the dipole scaling laws.

Output: Magnetic field vector  
B

  (Tesla) in ECI.

This approach provides a "physics-grade" environment that captures the modulation of the field seen by the satellite as it orbits, which is the primary signal the B-Dot controller exploits to damp angular momentum.

5. Rigid Body Dynamics: The Core Physics Engine
The heart of the simulation is the propagation of the satellite's rotational state. This requires solving the differential equations of rigid body motion. The snippet refers to files like test_dynamics.py, implying a modular separation of the dynamics solver.

5.1 The State Vector
To simulate the system, we must define the state vector X. For 6-DOF (Degree of Freedom) motion (though often only rotational 3-DOF is simulated for ADCS if orbit is decoupled), the state vector typically consists of 7 elements:

X=[q 
0
​
 ,q 
1
​
 ,q 
2
​
 ,q 
3
​
 ,ω 
x
​
 ,ω 
y
​
 ,ω 
z
​
 ] 
T
 
Quaternion (q): A 4-element vector representing the rotation from the Inertial Frame to the Body Frame. It is preferred over Euler angles to avoid singularities (Gimbal Lock). The scalar part is usually denoted q 
0
​
  or q 
w
​
 .   

Angular Velocity ( 
ω

 ): The rotation rate of the Body Frame with respect to the Inertial Frame, expressed in the Body Frame. This is crucial: Euler's moment equations are simplest when  
ω

  is in the body frame because the inertia tensor J is constant in that frame.

5.2 Inertia Tensor Specification
The dynamics depend heavily on the mass distribution, quantified by the Inertia Tensor J. $$ J = \begin{bmatrix} I_{xx} & -I_{xy} & -I_{xz} \ -I_{yx} & I_{yy} & -I_{yz} \ -I_{zx} & -I_{zy} & I_{zz} \end{bmatrix}

Forastandard3UCubeSat(approx3−4kg),thesnippet[25]providesrealisticvalues:
J_{3U} \approx \text{diag}(0.11, 0.0815, 0.0815) \text{ kg}\cdot\text{m}^2

Fora1UCubeSat(1kg),thevaluesaresmaller[26]:
J_{1U} \approx \text{diag}(0.002, 0.002, 0.002) \text{ kg}\cdot\text{m}^2 $$ A robust physics engine should allow the user to inject this matrix. Off-diagonal terms (I 
xy
​
 , etc.) should be non-zero in high-fidelity simulations to represent realistic manufacturing imperfections, which cause "wobble" (nutation) and coupling between axes.   

5.3 Euler's Equations of Motion (Dynamics)
Euler's equations relate the angular acceleration  
ω

 
˙
  to the applied torques  
τ

 . In the body frame:

τ

  
total
​
 =J 
ω

 
˙
 + 
ω

 ×(J 
ω

 )
Here,  
τ

  
total
​
  is the sum of all external torques (Magnetorquers, Aerodynamic, Gravity Gradient) and internal torques (Reaction Wheels, though B-Dot usually assumes magnetic only). The solver needs the derivative, so we rearrange:

ω

 
˙
 =J 
−1
 ( 
τ

  
total
​
 − 
ω

 ×(J 
ω

 ))
This equation must be evaluated at every time step by the SatellitePhysics module. The term  
ω

 ×(J 
ω

 ) is the gyroscopic coupling torque; it is responsible for phenomena like intermediate-axis instability (the Dzhanibekov effect), as highlighted in snippet.   

5.4 Quaternion Kinematics
We also need the derivative of the quaternion to update the orientation. The relationship between the quaternion derivative  
q
˙
​
  and the angular velocity  
ω

  is given by the kinematic differential equation :   

q
˙
​
 = 
2
1
​
 Ω( 
ω

 )q
Where Ω( 
ω

 ) is a skew-symmetric matrix formed from the angular velocity components: $$ \Omega(\vec{\omega}) = \begin{bmatrix} 0 & -\omega_x & -\omega_y & -\omega_z \ \omega_x & 0 & \omega_z & -\omega_y \ \omega_y & -\omega_z & 0 & \omega_x \ \omega_z & \omega_y & -\omega_x & 0 \end{bmatrix} $$ (Note: The sign convention depends on whether the quaternion is defined as [q 
w
​
 ,q 
x
​
 ,q 
y
​
 ,q 
z
​
 ] or [q 
x
​
 ,q 
y
​
 ,q 
z
​
 ,q 
w
​
 ]. The physics module must consistently enforce one convention, typically scalar-first or scalar-last to match libraries like numpy-quaternion).

6. Numerical Integration: The Runge-Kutta Implementation
With the derivatives defined ( 
q
˙
​
  and  
ω

 
˙
 ), the system is a set of coupled Ordinary Differential Equations (ODEs). To simulate time evolution, we must integrate these.

6.1 The Failure of Euler Integration
The simplest method, Euler integration (y 
n+1
​
 =y 
n
​
 +h⋅ 
y
˙
​
  
n
​
 ), is insufficient for orbital dynamics. Snippet  explicitly notes that Euler integration causes artificial energy growth (drift), causing the satellite to spin up or spiral out of orbit purely due to numerical error.   

6.2 Runge-Kutta 4 (RK4) Algorithm
The industry standard for this level of simulation is the 4th-Order Runge-Kutta (RK4) method. It is explicitly requested in the context of Python satellite dynamics. RK4 samples the derivative at four points within the time step dt:   

k 
1
​
  at the start (t).

k 
2
​
  at the midpoint (t+dt/2), using slope k 
1
​
 .

k 
3
​
  at the midpoint, using slope k 
2
​
 .

k 
4
​
  at the end (t+dt), using slope k 
3
​
 .

The weighted average is used for the update:

State 
new
​
 =State 
old
​
 + 
6
dt
​
 (k 
1
​
 +2k 
2
​
 +2k 
3
​
 +k 
4
​
 )
6.3 Quaternion Normalization
A critical detail for the "missing solver" file is quaternion normalization. Numerical integration errors (truncation error) will inevitably cause the quaternion's magnitude to drift away from 1 (∣q∣

=1). A non-unit quaternion does not represent a valid rotation and distorts the physics. The solver must include a normalization step after every integration update :   

q 
new
​
 ← 
∣∣q 
new
​
 ∣∣
q 
new
​
 
​
 
Failure to implement this leads to "skewing" of the satellite geometry in the simulation.

7. Control Theory: The B-Dot Algorithm
The user specifically requested the B-Dot controller files. Understanding the physics engine allows us to correctly implement the controller which closes the loop.

7.1 Theoretical Basis
The B-Dot controller is a kinetic energy management system. It does not target a specific attitude (pointing); it targets zero angular velocity ( 
ω

 →0). It relies on the time-derivative of the magnetic field vector in the body frame ( 
B

 
˙
  
body
​
 ). $$ \dot{\vec{B}}{body} = \frac{d}{dt} (A \vec{B}{inertial}) = \dot{A} \vec{B}{inertial} + A \dot{\vec{B}}{inertial}

$$Since $\dot{A} = -[\omega \times] A$, we get:$$
\dot{\vec{B}}{body} = -\vec{\omega} \times \vec{B}{body} + A \dot{\vec{B}}{inertial} $$ For a tumbling satellite, the rotation term (− 
ω

 × 
B

 ) dominates the orbital change ($A \dot{\vec{B}}{inertial}$). Thus,  
B

 
˙
  
body
​
  acts as a proxy for − 
ω

 .

7.2 The Control Law
To damp the motion, we generate a magnetic dipole  
m

  that creates a torque opposing the rotation.

m

 =−K 
B

 
˙
  
body
​
 
The resulting torque is:

τ

 = 
m

 × 
B

  
body
​
 =−K( 
B

 
˙
  
body
​
 × 
B

  
body
​
 )
Analysis shows this torque is always dissipative (extracts kinetic energy).   

7.3 Discrete Derivative and Noise
In the simulation (and on the satellite), we cannot access continuous  
B

 
˙
 . We must compute it digitally:

B

 
˙
  
sim
​
 [k]≈ 
Δt
B

  
meas
​
 [k]− 
B

  
meas
​
 [k−1]
​
 
The "missing control file" must implement this memory (storing last_B). Snippet  emphasizes that because the OBC (On-Board Computer) has no state variable in some Simulink models, the derivative must be explicitly handled. In Python, the Controller class must maintain this state.   

7.4 Gain Tuning and Units
One of the most common pitfalls identified in the research  is gain scaling.   

B

  is in Tesla (∼10 
−5
  T).

B

 
˙
  is in Tesla/second.

Max Dipole  
m

  
max
​
  for a CubeSat is ∼0.1−0.2 Am 
2
 .

This implies the gain K must be very large (order of 10 
4
  to 10 
6
 ) to produce meaningful torque from minute field changes.

Alternatively, a Bang-Bang B-Dot controller is used:

m

 =−m 
max
​
  sgn( 
B

 
˙
 )
This is simpler to tune and often more effective for initial high-speed detumbling.   

8. Python Implementation: The Missing Modules
Based on the theoretical derivations above, we can now reconstruct the specific Python classes required to "enable this to work." The code structure mirrors the architecture of standard GNC libraries like PyCubed or PyGNC.   

8.1 environment.py: The Magnetic Model
This module implements the Tilted Dipole model derived in Section 4.

Python
import numpy as np

class MagneticModel:
    """
    Implements the Tilted Dipole Model for Earth's Magnetic Field.
    Provides the magnetic field vector in the ECI (Inertial) frame.
    """
    def __init__(self):
        # Physical Constants
        self.Re = 6371000.0  # Earth Mean Radius (m)
        self.B0 = 3.12e-5    # Equatorial Field Strength (Tesla)
        self.mu_e = self.B0 * (self.Re ** 3) # Earth's Magnetic Moment
        
        # Dipole Parameters
        self.dipole_tilt = np.deg2rad(11.5) # Tilt relative to rotation axis
        self.w_earth = 7.2921159e-5 # Earth rotation rate (rad/s)
        
    def get_field_ECI(self, r_eci, t_seconds):
        """
        Calculate B-field in ECI frame at position r_eci and time t.
        
        Args:
            r_eci (np.array): Position vector [x, y, z] in meters.
            t_seconds (float): Simulation time in seconds.
            
        Returns:
            np.array: Magnetic field vector in Tesla.
        """
        r_norm = np.linalg.norm(r_eci)
        if r_norm < self.Re:
            raise ValueError("Satellite is underground!")

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
        # B = (mu_e / r^5) * (3(m. r)r - r^2 * m)
        
        dot_product = np.dot(m_hat, r_eci)
        
        term1 = 3 * dot_product * r_eci
        term2 = (r_norm**2) * m_hat
        
        B_vec = (self.mu_e / (r_norm**5)) * (term1 - term2)
        
        return B_vec
8.2 satellite_physics.py: The Dynamics Engine
This module implements the rigid body dynamics and kinematics derived in Section 5.

Python
import numpy as np

class SatellitePhysics:
    """
    Rigid Body Physics Engine.
    Manages the state vector X = [q0, q1, q2, q3, wx, wy, wz].
    """
    def __init__(self, inertia_matrix):
        self.J = np.array(inertia_matrix)
        self.J_inv = np.linalg.inv(self.J)
        
    def normalize_quaternion(self, state):
        """Forces quaternion norm to 1 to prevent numerical drift."""
        q = state[0:4]
        norm = np.linalg.norm(q)
        if norm > 1e-9:
            state[0:4] = q / norm
        return state

    def state_derivative(self, t, state, applied_torque_body):
        """
        Computes X_dot = f(t, X, tau).
        Used by the RK4 integrator.
        """
        # Unpack State
        q = state[0:4]  # Quaternion
        w = state[4:7]  # Angular Velocity (rad/s)
        
        # --- Kinematics (q_dot) ---
        # q_dot = 0.5 * Omega * q
        # Construct the skew-symmetric matrix for quaternion update
        Omega = np.array([0,    -w, -w, -w],
            [w,  0,     w, -w],
            [w, -w,  0,     w],
            [w,  w, -w
        ])
        q_dot = 0.5 * np.dot(Omega, q)
        
        # --- Dynamics (w_dot) ---
        # Euler's Equation: J w_dot + w x Jw = Torque
        # w_dot = J_inv * (Torque - w x Jw)
        
        Jw = np.dot(self.J, w)
        gyroscopic_torque = np.cross(w, Jw)
        
        net_torque = applied_torque_body - gyroscopic_torque
        w_dot = np.dot(self.J_inv, net_torque)
        
        # Combine derivatives
        state_dot = np.concatenate((q_dot, w_dot))
        return state_dot
8.3 solver.py: The Integrator
This implements the RK4 solver derived in Section 6.

Python
class RK4Solver:
    """Runge-Kutta 4th Order Numerical Integrator."""
    def __init__(self, physics_engine):
        self.physics = physics_engine
        
    def step(self, t, state, dt, torque):
        """
        Advances state by dt using RK4.
        """
        # k1
        k1 = dt * self.physics.state_derivative(t, state, torque)
        
        # k2
        k2 = dt * self.physics.state_derivative(t + 0.5*dt, state + 0.5*k1, torque)
        
        # k3
        k3 = dt * self.physics.state_derivative(t + 0.5*dt, state + 0.5*k2, torque)
        
        # k4
        k4 = dt * self.physics.state_derivative(t + dt, state + k3, torque)
        
        # Update
        new_state = state + (k1 + 2*k2 + 2*k3 + k4) / 6.0
        
        # Critical: Renormalize quaternion
        new_state = self.physics.normalize_quaternion(new_state)
        
        return new_state
8.4 control.py: The B-Dot Algorithm
This implements the control logic derived in Section 7.

Python
import numpy as np

class BDotController:
    """
    B-Dot Magnetic Detumbling Controller.
    """
    def __init__(self, gain, dt_control):
        self.k = gain
        self.dt = dt_control
        self.last_B_body = None
        self.max_dipole = 0.15 # Am^2 (Typical 1U magnetorquer)

    def compute_dipole(self, current_B_body):
        """
        Calculates dipole command based on B-field rate of change.
        """
        # Handle first step (no derivative possible)
        if self.last_B_body is None:
            self.last_B_body = current_B_body
            return np.zeros(3)
            
        # 1. Discrete Derivative
        b_dot = (current_B_body - self.last_B_body) / self.dt
        
        # 2. Control Law (Proportional)
        m_command = -self.k * b_dot
        
        # 3. Saturation (Actuator Limits)
        # Clip each axis to max dipole
        m_command = np.clip(m_command, -self.max_dipole, self.max_dipole)
        
        # Update memory
        self.last_B_body = current_B_body
        
        return m_command
9. Integration and Simulation Loop
To answer the user's request fully, one must show how these disparate files connect. The simulation main loop acts as the conductor.

Setup: Initialize Orbit, MagneticModel, SatellitePhysics, and BDotController.

Time Loop: Iterate t from 0 to T 
end
​
  with step dt.

Ephemeris: At each step, get r 
ECI
​
  from Orbit.

Environment: Get  
B

  
ECI
​
  from MagneticModel.

Sensor Transformation: Rotate  
B

  
ECI
​
  to  
B

  
Body
​
  using the current quaternion q from the state vector. This simulates the magnetometer reading.

Note: High-fidelity simulations would add Gaussian noise here (N(0,σ)) to test controller robustness.   

Control: If the control loop timer triggers (e.g., 1 Hz vs. Physics 10 Hz), call BDotController.compute_dipole.

Actuation: Calculate Torque  
τ

 = 
m

 × 
B

  
Body
​
 .

Physics Step: Call RK4Solver.step to advance the state using the calculated torque.

This closed-loop architecture ensures that the controller's actions (Dipole) interact with the environment (B-Field) to produce dynamics (Rotation), which in turn affects the sensor readings (B-Field in Body Frame), completing the feedback cycle.

10. Conclusion
The successful implementation of a CubeSat Attitude Control simulation requires more than just the control algorithm; it demands a rigorous reconstruction of the physical world. By implementing the SatellitePhysics dynamics engine, the MagneticModel environment, and the RK4Solver derived in this report, the missing "physics" modules of the Advitiy Control Model are fully realized. These components provide the mathematical fidelity required to validate the B-Dot controller, ensuring that the simulated detumbling behavior reflects the physical reality of orbital flight. The Python implementations provided offer a robust, extensible foundation for further development, capable of supporting advanced features like IGRF modeling, gravity gradient perturbations, and Hardware-in-the-Loop testing.


rhodesmill.org
API Reference — Earth Satellites — Skyfield documentation - Rhodes Mill
Opens in a new window

rhodesmill.org
Earth Satellites — Skyfield documentation - Rhodes Mill
Opens in a new window

uio.no
Control systems - UiO
Opens in a new window

webthesis.biblio.polito.it
Development of a ROS2 flight software framework & Attitude Control application for nanosatellites - Politecnico di Torino
Opens in a new window

prappleizer.github.io
Writing your own RK4 Orbit Integrator (Part 1: N Body) - Python for Astronomers
Opens in a new window

digitalrepository.unm.edu
The Quaternions with an application to Rigid Body Dynamics - UNM Digital Repository
Opens in a new window

elib.dlr.de
Design and Simulation of an Attitude Control System for a Microsatellite - electronic library -
Opens in a new window

s3vi.ndc.nasa.gov
Hardware-in-the-Loop and Software-in-the-Loop Testing of the MOVE-II CubeSat - S3VI
Opens in a new window

ri.cmu.edu
Optimization-based Methods for Satellite Control - Carnegie Mellon University Robotics Institute
Opens in a new window

rhodesmill.org
Coordinates — Skyfield documentation - Rhodes Mill
Opens in a new window

space.stackexchange.com
calculating satellite position and velocity from TLE - Space Exploration Stack Exchange
Opens in a new window

rhodesmill.org
Table of Contents — Skyfield documentation - Rhodes Mill
Opens in a new window

ieeexplore.ieee.org
Low Weight and Inertia Self-Balancing Testbed for a 3U CubeSat Attitude Control System - IEEE Xplore
Opens in a new window

science.gov
application visualization system: Topics by Science.gov
Opens in a new window

rhodesmill.org
API Reference — Skyfield documentation - Rhodes Mill
Opens in a new window

rhodesmill.org
Positions — Skyfield documentation - Rhodes Mill
Opens in a new window

arxiv.org
Developing and Implementing a CubeSat's Equations of Motion - arXiv
Opens in a new window

cedarscience.org
IGRF in Python - CEDAR
Opens in a new window

ngdc.noaa.gov
IAGA V-MOD Geomagnetic Field Modeling: International Geomagnetic Reference Field IGRF-13 - NOAA
Opens in a new window

arxiv.org
Modelling the Earth's magnetic field - arXiv
Opens in a new window

dokumen.pub
ADCS - Spacecraft Attitude Determination and Control 9780323999151 - DOKUMEN.PUB
Opens in a new window

scipython.com
Visualizing the Earth's dipolar magnetic field - Scientific Programming with Python
Opens in a new window

medium.com
Working with Quaternions in NumPy | by whyamit404 - Medium
Opens in a new window

arxiv.org
1 Introduction - arXiv
Opens in a new window

stackoverflow.com
Cannot get RK4 to solve for position of orbiting body in Python - Stack Overflow
Opens in a new window

d-nb.info
Improved quaternion-based integration scheme for rigid body motion
Opens in a new window

researchgate.net
(PDF) Simulation of CubeSat Detumbling Using B-Dot Controller - ResearchGate
Opens in a new window

repositorio.ufsc.br
CubeSat Detumbling Simulation Using B-dot Control Law
Opens in a new window
