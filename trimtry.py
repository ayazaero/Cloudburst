#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import numpy as np
from scipy.optimize import fsolve

# Given parameters
m = 13.5  # kg
g = 9.81  # m/s^2
S = 0.55  # m^2
b = 2.8956  # m
c = 0.18994  # m
Sprop = 0.2027  # m^2
rho_air = 1.2682  # kg/m^3
kmotor = 80
C_prop = 1.0
V = 30  # Airspeed in m/s
CL0 = 0.28
CDp = 0.0437
Cm0 = -0.02338
CL_alpha = 3.45
Cm_alpha = -0.38
Cm_delta_e = -0.5
alpha0 = 0.4712  # radians
M = 50  # shaping factor for the sigmoid function
AR = b**2 / S  # Aspect Ratio
e = 0.9  # Oswald efficiency factor

# Sigmoid function
def sigma(alpha):
    return (1 + np.exp(-M * (alpha - alpha0)) + np.exp(M * (alpha + alpha0))) / (
        (1 + np.exp(-M * (alpha - alpha0))) * (1 + np.exp(M * (alpha + alpha0)))
    )

# Lift coefficient calculation
def CL(alpha, delta_e):
    CL_linear = CL0 + CL_alpha * alpha
    CL_non_linear = 2 * np.sign(alpha) * np.sin(alpha)**2 * np.cos(alpha)
    return (1 - sigma(alpha)) * CL_linear + sigma(alpha) * CL_non_linear

# Drag coefficient calculation
def CD(alpha, delta_e):
    CL_total = CL(alpha, delta_e)
    return CDp + (CL_total**2) / (np.pi * e * AR)

# Propeller force function
def prop_force(V, delta_t):
    return 0.5 * rho_air * Sprop * C_prop * ((kmotor * delta_t)**2 - V**2)

# Function to solve for trim conditions
def trim_equations(variables):
    alpha, delta_e, delta_t, theta = variables
    
    # Lift coefficient
    CL_val = CL(alpha, delta_e)
    
    # Drag coefficient
    CD_val = CD(alpha, delta_e)
    
    # Lift force
    L = 0.5 * rho_air * V**2 * S * CL_val
    
    # Drag force
    D = 0.5 * rho_air * V**2 * S * CD_val
    
    # Weight
    W = m * g
    
    # Thrust force
    T = prop_force(V, delta_t)
    
    # Equilibrium equations
    fx = (-D * np.cos(alpha)) + (L * np.sin(alpha)) - (m * g * np.sin(theta)) + T
    fy = (-D * np.sin(alpha)) - (L * np.cos(alpha)) + (m * g * np.cos(theta))
    M_pitch = 0.5 * rho_air * V**2 * S * c * (Cm0 + Cm_alpha * alpha + Cm_delta_e * delta_e)  # Pitch moment should be zero
    
    # Fourth equation: theta should be equal to alpha (steady level flight assumption)
    theta_eq = theta - alpha
    
    return [fx, fy, M_pitch, theta_eq]

# Initial guesses for alpha (radians), delta_e (radians), delta_t (throttle setting), and theta (radians)
initial_guess = [0.1, -0.1, 0.5, 0.1]

# Solve for trim conditions
trim_solution = fsolve(trim_equations, initial_guess)

# Extract results
alpha_trim = np.degrees(trim_solution[0])  # Convert to degrees
delta_e_trim = np.degrees(trim_solution[1])  # Convert to degrees
delta_t_trim = trim_solution[2]
theta_trim = np.degrees(trim_solution[3])  # Convert to degrees

# Output the results
print(f"Trim angle of attack (alpha): {alpha_trim:.4f} degrees")
print(f"Trim elevator deflection (delta_e): {delta_e_trim:.4f} degrees")
print(f"Trim throttle setting (delta_t): {delta_t_trim:.4f}")
print(f"Trim pitch angle (theta): {theta_trim:.4f} degrees")
