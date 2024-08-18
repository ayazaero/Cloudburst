# Aerodynamic coefficients for the Aerosonde UAV

# Mass and inertia parameters
m = 13.5  # kg
J_x = 0.8244  # kg-m^2
J_y = 1.135  # kg-m^2
J_z = 1.759  # kg-m^2
J_xz = 0.1204  # kg-m^2

# Geometry parameters
S = 0.55  # m^2
b = 2.8956  # m
c = 0.18994  # m
Sprop = 0.2027  # m^2
rho_air = 1.2682  # kg/m^3

# Motor parameters
kmotor = 80
kTp = 0
k_omega = 0  # Assuming 'k' corresponds to k_phi
e = 0.9

# Aerodynamic coefficients
alpha0 = 0.4712
alpha_dot = 0.1592  # Assuming '' corresponds to alpha_dot
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
