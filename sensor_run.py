#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import numpy as np
import time
import threading
from pyproj import Proj, Transformer
import wmm2020
from datetime import datetime, timezone
import queue
import zmq
import os
import sensor_parameters as sp

def get_free_port(starting_port=5557, max_attempts=100):
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    port = starting_port
    attempts = 0

    while attempts < max_attempts:
        try:
            print('Trying to bind to port:', port)
            socket.bind(f"tcp://127.0.0.1:{port}")
            socket.unbind(f"tcp://127.0.0.1:{port}")
            print('Successfully bound and unbound port:', port)
            break
        except zmq.ZMQError:
            port += 1
            attempts += 1
            print(f'Port {port - 1} in use, trying next. Attempt: {attempts}')
    else:
        raise RuntimeError(f"Could not find a free port after {max_attempts} attempts.")
    
    socket.close()
    context.term()
    print('Context terminated.')

    print('Found free port:', port)
    return port


class Sensors:
    def __init__(self,agent_id,sense_address=None):
        self.agent_id=agent_id
        self.context = zmq.Context()
        
        # Sensor Parameters
        self.accel_sigma = sp.accel_sigma  # standard deviation of accelerometers in m/s^2
        self.accel_rate = sp.accel_rate  # Update rate in seconds (100 Hz)

        #-------- Rate Gyro --------
        self.gyro_x_bias = sp.gyro_x_bias  # bias on x_gyro
        self.gyro_y_bias = sp.gyro_y_bias  # bias on y_gyro
        self.gyro_z_bias = sp.gyro_z_bias  # bias on z_gyro
        self.gyro_sigma = sp.gyro_sigma  # standard deviation of gyros in rad/sec
        self.gyro_rate = sp.gyro_rate  # Update rate in seconds (50 Hz)

        #-------- Pressure Sensor (Altitude) --------
        self.abs_pres_sigma = sp.abs_pres_sigma  # standard deviation of absolute pressure sensors in Pascals
        self.altimeter_rate = sp.altimeter_rate  # Update rate in seconds (20 Hz)

        #-------- Pressure Sensor (Airspeed) --------
        self.diff_pres_sigma = sp.diff_pres_sigma  # standard deviation of diff pressure sensor in Pascals
        self.pitot_rate = sp.pitot_rate  # Update rate in seconds (20 Hz)

        #-------- Magnetometer --------
        self.mag_beta = sp.mag_beta
        self.mag_sigma = sp.mag_sigma
        self.mag_rate = sp.mag_rate  # Update rate in seconds (10 Hz)

        #-------- 2017 GPS --------
        self.kGPS = sp.kGPS  # Time constant of the process
        self.Ts = sp.Ts  # Sample time
        self.gps_n_sigma = sp.gps_n_sigma
        self.gps_e_sigma = sp.gps_e_sigma
        self.gps_h_sigma = sp.gps_h_sigma
        self.gps_Vg_sigma = sp.gps_Vg_sigma
        self.gps_course_sigma = sp.gps_course_sigma
        self.gps_rate = sp.gps_rate  # Update rate in seconds



        # Initialize errors
        self.error_n = 0.0
        self.error_e = 0.0
        self.error_h = 0.0

        self.sensor_data = {}
        self.sensor_data_queue = queue.Queue()
        self.running = True

        if sense_address is None:
            sense_port = get_free_port(5560)
            sense_address = f"tcp://127.0.0.1:{sense_port}"
            print(f"{self.agent_id}: Binding sensor PUB socket to {sense_address}")

        self.sense_address = sense_address
        self.sense_socket = self.context.socket(zmq.PUB)
        self.sense_socket.bind(self.sense_address)

        print(f"{self.agent_id}: Adding agent to the file")
        self.add_agent_to_file()
        print(f"{self.agent_id}: Added agent to the file")
        

    def simulate_accelerometer(self, true_acceleration):
        noise = np.random.normal(0, self.accel_sigma, size=true_acceleration.shape)
        return (true_acceleration + noise).squeeze()

    def simulate_gyro(self, true_angular_velocity):
        true_angular_velocity=true_angular_velocity.squeeze()
        biases = np.array([self.gyro_x_bias, self.gyro_y_bias, self.gyro_z_bias]).squeeze()
        noise = np.random.normal(0, self.gyro_sigma, size=true_angular_velocity.shape)
        return true_angular_velocity + biases + noise

    def simulate_altimeter(self, true_altitude):
        noise = np.random.normal(0, self.abs_pres_sigma)
        return (true_altitude + noise).squeeze()

    def simulate_pitot_tube(self, true_airspeed):
        noise = np.random.normal(0, self.diff_pres_sigma)
        #print(true_airspeed)
        return (true_airspeed + noise).squeeze()

    def simulate_magnetometer(self, true_magnetic_field):
        noise = np.random.normal(0, np.degrees(self.mag_sigma), size=true_magnetic_field.shape)
        return true_magnetic_field + np.degrees(self.mag_beta) + noise

    def simulate_gps(self, true_position, true_velocity, true_course):
        true_position=true_position.squeeze()
        true_course=true_course.squeeze()
        true_velocity=true_velocity.squeeze()
        
        # Update errors using the Gauss-Markov process
        noise_n = np.random.normal(0, self.gps_n_sigma)
        noise_e = np.random.normal(0, self.gps_e_sigma)
        noise_h = np.random.normal(0, self.gps_h_sigma)

        self.error_n = np.exp(-self.kGPS * self.Ts) * self.error_n + noise_n
        self.error_e = np.exp(-self.kGPS * self.Ts) * self.error_e + noise_e
        self.error_h = np.exp(-self.kGPS * self.Ts) * self.error_h + noise_h

        # Generate GPS measurements with errors
        simulated_position = true_position + np.array([self.error_n, self.error_e, -self.error_h])  # Convert pd to positive altitude
         # Calculate horizontal ground speed
        Vn = true_velocity[0]  # North velocity component
        Ve = true_velocity[1]  # East velocity component
        Vg = np.sqrt(Vn**2 + Ve**2)  # Ground speed
        chi = np.arctan2(Vn, Ve)  # Course (heading)

        # Add noise to ground speed and course
        noise_Vg = np.random.normal(0, self.gps_Vg_sigma)
        noise_chi = np.random.normal(0, self.gps_course_sigma)

        simulated_velocity = Vg + noise_Vg  # Ground speed with noise
        simulated_course = chi + noise_chi    # Course with noise

        return simulated_position, simulated_velocity, simulated_course*180/np.pi  # Return position, ground speed, and course


    def run_sensor(self, sensor_func, rate, truestate, ref_lla, is_gps=False):
        while self.running:
            # Determine the correct state to pass to the sensor function
            if sensor_func == self.simulate_accelerometer:
                sensor_data = self.get_sensor_data(sensor_func, is_gps, truestate.sdot, ref_lla)
            elif sensor_func == self.simulate_pitot_tube:
                sensor_data = self.get_sensor_data(sensor_func, is_gps, truestate.state, truestate.Vwi)
            else:
                sensor_data = self.get_sensor_data(sensor_func, is_gps, truestate.state, ref_lla)

            timestamp = time.time()
            sensor_name = sensor_func.__name__
            message = f"{sensor_name} {timestamp} {sensor_data}"
            self.sense_socket.send_string(message)  # Publish data

            time.sleep(rate)
        print(f"Sensor thread for {sensor_func.__name__} exiting.")

    def get_sensor_data(self, sensor_func, is_gps, state,ref_lla):
        # Unpacking the state vector
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state

        if sensor_func == self.simulate_accelerometer:
            # True acceleration in the body frame based on Newton's laws
            true_acceleration = np.array([u, v, w])  # this function is actually getting udot, vdot and wdot from the call
            #print(true_acceleration)
            return self.simulate_accelerometer(true_acceleration)

        elif sensor_func == self.simulate_gyro:
            # Simulate the true angular velocity using body rates
            true_angular_velocity = np.array([p, q, r])  # Body rates
            return self.simulate_gyro(true_angular_velocity)

        elif sensor_func == self.simulate_altimeter:
            # Use negative altitude (pd) for the true altitude
            true_altitude = -pd
            return self.simulate_altimeter(true_altitude)

        elif sensor_func == self.simulate_pitot_tube:
            Vwi=ref_lla
            Rmat=self.rotation_matrix(phi,theta,psi)
            #print(Rmat)
            #print(Vwi)  
            Vwb=Rmat@Vwi
            
            Vgb=np.array([[u],[v],[w]])
            Vab=Vgb-Vwb

            # Calculate true airspeed based on the magnitude of body frame velocities
            true_airspeed = np.linalg.norm(Vab)
            return self.simulate_pitot_tube(true_airspeed)

        elif sensor_func == self.simulate_magnetometer:
            # Assuming a simplified magnetic field; adjust as needed
            lat,lon,alt = self.ned_to_lla([pn,pe,pd],ref_lla)
            B_ned,declination=self.get_magnetic_field_ned(lat,lon,alt)
            B_body = self.ned_to_body_frame(B_ned, phi, theta, psi)
            heading = self.calculate_heading_from_magnetometer(B_body)
            #print('declination:' ,declination.values.item())
            psi_mag = self.simulate_magnetometer(heading+declination)

            return psi_mag

        elif sensor_func == self.simulate_gps and is_gps:
            # Use the inertial positions (pn, pe, pd) for the true GPS position
            true_position = np.array([pn, pe, -pd])  # Convert pd to positive altitude
            true_velocity = np.array([u, v, w])  # Assuming body frame velocity for GPS
            true_course = psi  # Assuming heading as course
            return self.simulate_gps(true_position, true_velocity, true_course)

        else:
            raise ValueError("Unknown sensor function.")


    def start_sensor_thread(self, sensor_func, rate,state, ref_lla, is_gps=False):
        thread = threading.Thread(target=self.run_sensor, args=(sensor_func, rate,state,ref_lla, is_gps))
        thread.start()
        return thread

    def stop(self):
        self.running = False
        time.sleep(0.5)
        print(f"{self.agent_id}: Removing agent from the file")
        self.remove_agent_from_file()
        print(f"{self.agent_id}: Removed agent from the file")

    def add_agent_to_file(self):
        with open("agents_sensors.txt", "a") as file:
            file.write(f"{self.agent_id},{self.sense_address}\n")

    def remove_agent_from_file(self):
        if not os.path.exists("agents_sensors.txt"):
            return
        with open("agents_sensors.txt", "r") as file:
            lines = file.readlines()
        with open("agents_sensors.txt", "w") as file:
            for line in lines:
                if not line.startswith(self.agent_id):
                    file.write(line)

    def ned_to_lla(self,ned, ref_lla):
        # Define the WGS-84 ellipsoid
        wgs84 = Proj(proj='latlong', datum='WGS84')

        # Convert reference LLA to ECEF
        ref_lat, ref_lon, ref_alt = ref_lla
        ecef_proj = Proj(proj='geocent', datum='WGS84')
        transformer = Transformer.from_proj(wgs84, ecef_proj)
        ref_x, ref_y, ref_z = transformer.transform(ref_lon, ref_lat, ref_alt)
        

        # Calculate rotation matrix from NED to ECEF
        lat_rad = np.radians(ref_lat)
        lon_rad = np.radians(ref_lon)


        R = np.array([
            [-np.sin(lon_rad), -np.sin(lat_rad) * np.cos(lon_rad), np.cos(lat_rad) * np.cos(lon_rad)],
            [ np.cos(lon_rad), -np.sin(lat_rad) * np.sin(lon_rad), np.cos(lat_rad) * np.sin(lon_rad)],
            [ 0, np.cos(lat_rad), np.sin(lat_rad)]
        ])

        # Transform NED to ECEF
        ned_vector = ned
        ecef_vector = np.array([ref_x, ref_y, ref_z]) + R.dot(ned_vector).squeeze()
        
        # Convert ECEF to LLA
        transformer_inverse = Transformer.from_proj(ecef_proj, wgs84)
        lon, lat, alt = transformer_inverse.transform(*ecef_vector)
        #print(lat,lon,alt)
        return lat, lon, alt
    
    def get_magnetic_field_ned(self,lat, lon, alt, date=None):
        # If no date is provided, use the current UTC date and time
        if date is None:
            date = datetime.now(timezone.utc)

        # Calculate the decimal year
        yeardec = self.calculate_decimal_year(date)

        # Convert altitude from meters to kilometers as expected by the model
        alt_km = alt / 1000.0

        # Check if lat and lon are scalars
        if isinstance(lat, (list, np.ndarray)):
            lat = lat[0]  # Use the first element if it's a list or array
        if isinstance(lon, (list, np.ndarray)):
            lon = lon[0]  # Use the first element if it's a list or array
        if isinstance(alt_km, (list, np.ndarray)):
            alt_km = alt_km[0]  # Use the first element if it's a list or array

        # Get the magnetic field components in NED frame
        mag_field = wmm2020.wmm(lat, lon, alt_km, yeardec)

        declination = mag_field.decl

        # The wmm function returns the magnetic field in nanoteslas (nT) for the N, E, D components
        B_ned = np.array([mag_field.north, mag_field.east, mag_field.down]).squeeze()
        
        return B_ned, declination.values.item()
    
    def ned_to_body_frame(self,B_ned, phi, theta, psi):
        # Convert Euler angles (phi, theta, psi) to a rotation matrix
        R = self.euler_to_rotation_matrix(phi, theta, psi)
        # Rotate the NED magnetic field to the body frame
        B_body = R @ B_ned
        return B_body

    def euler_to_rotation_matrix(self,phi, theta, psi):
        phi=np.radians(phi[0])
        theta = np.radians(theta[0])
        psi = np.radians(psi[0])
        # Rotation matrix from NED to body frame
        Rz = np.array([[np.cos(psi), np.sin(psi), 0],
                    [-np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
        Ry = np.array([[np.cos(theta), 0, -np.sin(theta)],
                    [0, 1, 0],
                    [np.sin(theta), 0, np.cos(theta)]])
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(phi), np.sin(phi)],
                    [0, -np.sin(phi), np.cos(phi)]])
        return  Ry.T @ Rx.T @ Rz @ Ry @ Rx #find out why?
    
    def rotation_matrix(self,phi, theta, psi):
        phi=np.radians(phi[0])
        theta = np.radians(theta[0])
        psi = np.radians(psi[0])
        # Rotation matrix from NED to body frame
        Rz = np.array([[np.cos(psi), np.sin(psi), 0],
                    [-np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
        Ry = np.array([[np.cos(theta), 0, -np.sin(theta)],
                    [0, 1, 0],
                    [np.sin(theta), 0, np.cos(theta)]])
        Rx = np.array([[1, 0, 0],
                    [0, np.cos(phi), np.sin(phi)],
                    [0, -np.sin(phi), np.cos(phi)]])
        return  Rz @ Ry @ Rx

    def calculate_heading_from_magnetometer(self,B_measured):
        # Calculate heading from the measured magnetic field in the body frame
        heading = -np.arctan2(B_measured[1], B_measured[0])
        return np.degrees(heading)  # Convert to degrees
    
    def calculate_decimal_year(self,date):
        year = date.year
        start_of_year = datetime(year, 1, 1, tzinfo=timezone.utc)
        start_of_next_year = datetime(year + 1, 1, 1, tzinfo=timezone.utc)
        year_elapsed = (date - start_of_year).total_seconds()
        year_duration = (start_of_next_year - start_of_year).total_seconds()
        return year + year_elapsed / year_duration


