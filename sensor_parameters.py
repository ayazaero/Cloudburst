import numpy as np
accel_sigma = 0.0025 * 9.81  # standard deviation of accelerometers in m/s^2
accel_rate = 0.01  # Update rate in seconds (100 Hz)

#-------- Rate Gyro --------
gyro_x_bias = np.radians(5 * np.random.uniform(-1, 1))  # bias on x_gyro
gyro_y_bias = np.radians(5 * np.random.uniform(-1, 1))  # bias on y_gyro
gyro_z_bias = np.radians(5 * np.random.uniform(-1, 1))  # bias on z_gyro
gyro_sigma = np.radians(0.13)  # standard deviation of gyros in rad/sec
gyro_rate = 0.02  # Update rate in seconds (50 Hz)

#-------- Pressure Sensor (Altitude) --------
abs_pres_sigma = 0.01 * 1000  # standard deviation of absolute pressure sensors in Pascals
altimeter_rate = 0.05  # Update rate in seconds (20 Hz)

#-------- Pressure Sensor (Airspeed) --------
diff_pres_sigma = 0.002 * 1000  # standard deviation of diff pressure sensor in Pascals
pitot_rate = 0.05  # Update rate in seconds (20 Hz)

#-------- Magnetometer --------
mag_beta = np.radians(1.0)
mag_sigma = np.radians(0.03)
mag_rate = 0.1  # Update rate in seconds (10 Hz)

#-------- 2017 GPS --------
kGPS = 1/1100  # Time constant of the process
Ts = 1.0      # Sample time
gps_n_sigma = 0.01
gps_e_sigma = 0.01
gps_h_sigma = 0.03
gps_Vg_sigma = 0.005
gps_course_sigma = gps_Vg_sigma / 20
gps_rate = Ts  # Update rate in seconds


