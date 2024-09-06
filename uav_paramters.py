#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

# Aerodynamic coefficients for the Aerosonde UAV
import numpy as np

# Mass and inertia parameters
m = 13.5  # kg
J_x = 0.8244  # kg-m^2
J_y = 1.135  # kg-m^2
J_z = 1.759  # kg-m^2
J_xz = 0.1204  # kg-m^2
g=9.81

# Geometry parameters
S = 0.55  # m^2
b = 2.8956  # m
c = 0.18994  # m
Sprop = 0.2027  # m^2
rho_air = 1.2682  # kg/m^3

# Motor parameters
kmotor = 80
kTp = 0
k_omega = 0  # Assuming 'k' corresponds to k_phi
e = 0.9

# Aerodynamic coefficients
alpha0 = 0.4712
epsilon = 0.1592  # Assuming  corresponds to alpha_dot
CDp = 0.0437

# Longitudinal coefficients
CL0 = 0.28
CD0 = 0.03
Cm0 = -0.02338
CL_alpha = 3.45
CD_alpha = 0.30
Cm_alpha = -0.38
CL_q = 0
CD_q = 0
Cm_q = -3.6
CL_delta_e = -0.36
CD_delta_e = 0
Cm_delta_e = -0.5
C_prop = 1.0
M = 50.0


# Lateral coefficients
CY0 = 0
Cl0 = 0
Cn0 = 0
CY_beta = -0.98
Cl_beta = -0.12
Cn_beta = 0.25
CY_p = 0
Cl_p = -0.26
Cn_p = 0.022
CY_r = 0
Cl_r = 0.14
Cn_r = -0.35
CY_delta_a = 0
Cl_delta_a = 0.08
Cn_delta_a = 0.06
CY_delta_r = -0.17
Cl_delta_r = 0.105
Cn_delta_r = -0.03


# Tau parameters
C = (J_x * J_z) - J_xz**2
C1 = (J_xz * (J_x - J_y + J_z)) / C
C2 = (J_z * (J_z - J_y) + J_xz**2) / C
C3 = J_z / C
C4 = J_xz / C
C5 = (J_z - J_x) / J_y
C6 = (J_xz / J_y)
C7 = ((J_x - J_y) * J_x + J_xz**2) / C
C8 = J_x / C

# Max actuator limits
max_delta_a_l = 30*np.pi/180
max_delta_a_r = 30*np.pi/180
max_delta_a=30*np.pi/180
max_delta_r = 30*np.pi/180
max_delta_e = 30*np.pi/180
max_delta_t = 0.5
