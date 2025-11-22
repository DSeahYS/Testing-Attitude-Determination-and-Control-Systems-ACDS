"""
RK4 Numerical Integrator

Implements Section 6 of the 6DOF research document.
4th-Order Runge-Kutta method with fixed time step,
matching the deterministic behavior of flight software.
"""

import numpy as np

class RK4Solver:
    """
    Runge-Kutta 4th Order Numerical Integrator.
    
    RK4 is preferred over simple Euler integration because it prevents
    artificial energy growth and drift in orbital dynamics.
    
    The fixed time step matches embedded systems on real satellites
    and provides deterministic behavior for verification.
    """
    
    def __init__(self, physics_engine):
        """
        Initialize solver with physics engine.
        
        Args:
            physics_engine: Object with state_derivative(t, state, torque) method
        """
        self.physics = physics_engine
        
    def step(self, t, state, dt, torque):
        """
        Advances state by dt using 4th-order Runge-Kutta.
        
        Algorithm:
            k1 = f(t, x)
            k2 = f(t + dt/2, x + k1·dt/2)
            k3 = f(t + dt/2, x + k2·dt/2)
            k4 = f(t + dt, x + k3·dt)
            x_new = x + (dt/6)(k1 + 2k2 + 2k3 + k4)
        
        Args:
            t (float): Current time (seconds)
            state (np.array): Current state [q, ω]
            dt (float): Time step (seconds)
            torque (np.array): Applied torque [τx, τy, τz] in N·m
            
        Returns:
            np.array: New state after time dt
        """
        # k1: Derivative at start
        k1 = dt * self.physics.state_derivative(t, state, torque)
        
        # k2: Derivative at midpoint using k1
        k2 = dt * self.physics.state_derivative(
            t + 0.5*dt, 
            state + 0.5*k1, 
            torque
        )
        
        # k3: Derivative at midpoint using k2
        k3 = dt * self.physics.state_derivative(
            t + 0.5*dt, 
            state + 0.5*k2, 
            torque
        )
        
        # k4: Derivative at end using k3
        k4 = dt * self.physics.state_derivative(
            t + dt, 
            state + k3, 
            torque
        )
        
        # Weighted average update
        new_state = state + (k1 + 2*k2 + 2*k3 + k4) / 6.0
        
        # Critical: Renormalize quaternion to prevent drift
        new_state = self.physics.normalize_quaternion(new_state)
        
        return new_state
    
    def integrate_trajectory(self, t_start, t_end, dt, initial_state, torque_func):
        """
        Integrate over a time span with time-varying torque.
        
        Useful for batch simulations and analysis.
        
        Args:
            t_start (float): Start time
            t_end (float): End time
            dt (float): Time step
            initial_state (np.array): Initial state [q, ω]
            torque_func: Function torque = f(t, state) returning torque vector
            
        Returns:
            tuple: (time_array, state_history)
        """
        time_points = np.arange(t_start, t_end, dt)
        state_history = np.zeros((len(time_points), len(initial_state)))
        
        state = initial_state.copy()
        
        for i, t in enumerate(time_points):
            state_history[i] = state
            torque = torque_func(t, state)
            state = self.step(t, state, dt, torque)
            
        return time_points, state_history
