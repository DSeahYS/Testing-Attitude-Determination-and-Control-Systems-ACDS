"""
Rigid Body Dynamics Engine

Implements Section 5 of the 6DOF research document.
Manages the 7-element state vector [q0, q1, q2, q3, ωx, ωy, ωz]
and computes the state derivatives for numerical integration.
"""

import numpy as np

class SatellitePhysics:
    """
    Rigid Body Physics Engine.
    
    State Vector: X = [q0, q1, q2, q3, wx, wy, wz]
    - Quaternion (q): Rotation from ECI to Body frame (scalar-first convention)
    - Angular Velocity (ω): Body frame angular velocity (rad/s)
    """
    
    def __init__(self, inertia_matrix):
        """
        Initialize physics engine with satellite inertia.
        
        Args:
            inertia_matrix (array-like): 3x3 inertia tensor in kg·m²
                For 3U CubeSat: diag(0.11, 0.0815, 0.0815)
                For 1U CubeSat: diag(0.002, 0.002, 0.002)
        """
        self.J = np.array(inertia_matrix, dtype=np.float64)
        self.J_inv = np.linalg.inv(self.J)
        
    def normalize_quaternion(self, state):
        """
        Forces quaternion norm to 1 to prevent numerical drift.
        
        Critical step: RK4 integration causes quaternion magnitude
        to drift from unity, which distorts the physics.
        
        Args:
            state (np.array): State vector [q, ω]
            
        Returns:
            np.array: State with normalized quaternion
        """
        q = state[0:4]
        norm = np.linalg.norm(q)
        if norm > 1e-9:
            state[0:4] = q / norm
        return state

    def state_derivative(self, t, state, applied_torque_body):
        """
        Computes state derivative: X_dot = f(t, X, τ)
        
        Used by the RK4 integrator to propagate dynamics.
        
        Equations:
            1. Kinematics: q̇ = 0.5 * Ω(ω) * q
            2. Dynamics: ω̇ = J⁻¹[τ - ω×(Jω)]
        
        Args:
            t (float): Current time (not used, included for integrator compatibility)
            state (np.array): Current state [q0, q1, q2, q3, wx, wy, wz]
            applied_torque_body (np.array): External torque [τx, τy, τz] in N·m
            
        Returns:
            np.array: State derivative [q̇, ω̇]
        """
        # Unpack State
        q = state[0:4]  # Quaternion [qw, qx, qy, qz]
        w = state[4:7]  # Angular Velocity (rad/s)
        
        # --- Kinematics (q_dot) ---
        # q̇ = 0.5 * Ω * q
        # Construct the skew-symmetric matrix for quaternion update
        # Using scalar-first convention [qw, qx, qy, qz]
        Omega = np.array([
            [0,    -w[0], -w[1], -w[2]],
            [w[0],  0,     w[2], -w[1]],
            [w[1], -w[2],  0,     w[0]],
            [w[2],  w[1], -w[0],  0   ]
        ])
        q_dot = 0.5 * np.dot(Omega, q)
        
        # --- Dynamics (w_dot) ---
        # Euler's Equation: J·ω̇ + ω×(J·ω) = τ
        # Solving for ω̇: ω̇ = J⁻¹[τ - ω×(J·ω)]
        
        Jw = np.dot(self.J, w)
        gyroscopic_torque = np.cross(w, Jw)
        
        net_torque = applied_torque_body - gyroscopic_torque
        w_dot = np.dot(self.J_inv, net_torque)
        
        # Combine derivatives
        state_dot = np.concatenate((q_dot, w_dot))
        return state_dot
    
    def quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion to rotation matrix (DCM).
        
        Useful for transforming vectors from ECI to Body frame.
        
        Args:
            q (np.array): Quaternion [qw, qx, qy, qz]
            
        Returns:
            np.array: 3x3 rotation matrix (ECI -> Body)
        """
        qw, qx, qy, qz = q
        
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qw*qz, 2*qx*qz + 2*qw*qy],
            [2*qx*qy + 2*qw*qz, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qw*qx],
            [2*qx*qz - 2*qw*qy, 2*qy*qz + 2*qw*qx, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        return R
    
    def rotate_vector_by_quaternion(self, q, v):
        """
        Rotate a vector by quaternion (alternative to DCM method).
        
        Uses the formula: v' = v + 2q_vec × (q_vec × v + qw·v)
        
        Args:
            q (np.array): Quaternion [qw, qx, qy, qz]
            v (np.array): Vector to rotate [x, y, z]
            
        Returns:
            np.array: Rotated vector
        """
        q_vec = q[1:]  # [qx, qy, qz]
        q_scalar = q[0]  # qw
        
        t = 2 * np.cross(q_vec, v)
        v_prime = v + q_scalar * t + np.cross(q_vec, t)
        
        return v_prime
