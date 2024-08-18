clc
clear

%% Aircraft parameters - Zagi

% Zagi Flying Wing Parameters
m = 1.56; % kg
Jx = 0.1147; % kg.m^2
Jy = 0.0576; % kg.m^2
Jz = 0.1712; % kg.m^2
Jxz = 0.0015; % kg.m^2
S = 0.2589; % m^2
b = 1.4224; % m
c = 0.3302; % m

% Longitudinal Coefficients
C_L0 = 0.09167;
C_D0 = 0.01631;
C_m0 = -0.02338;
C_Lalpha = 3.5016;
C_Dalpha = -0.2108;
C_malpha = -0.5675;
C_Lq = 2.8932;
C_Dq = 0;
C_mq = -0.00040;

% Lateral Coefficients
C_Y0 = 0;
C_l0 = 0;
C_n0 = 0;
C_Ybeta = -0.07359;
C_lbeta = 0.02054;
C_nbeta = 0.00040;
C_Yp = 0;
C_lp = -0.5075;
C_np = -0.00040;
C_Yr = 0;
C_lr = 0.02054;
C_nr = -0.00040;

c = 0.3302; % m
S = 0.314; % m^2
rho = 1.2682; % kg/m^3
e = 0.9;
AR = 20;

% Longitudinal Coefficients
C_L0 = -1.3990;
C_D0 = 0.2724;
C_m0 = -0.03209;
C_Lalpha = 3.245;
C_Dalpha = -0.3254;
C_malpha = -0.3309;
C_Lq = 0.03066;
C_Dq = 0.04712;
C_mq = -0.00328;

% Lateral Coefficients
C_Y0 = 0;
C_l0 = 0;
C_n0 = 0;
C_Ybeta = 0.1297;
C_lbeta = 0.3066;
C_nbeta = 0.0254;
C_Yp = -0.3209;
C_lp = 0.03066;
C_np = 0.000434;
C_Yr = 0.4712;
C_lr = -0.00328;
C_nr = -0.000434;
