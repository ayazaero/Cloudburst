#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import numpy as np
import time
import threading
from pyproj import Proj, Transformer

class Atmos:
    def __init__(self):
        self.wind_scale = np.array([0.5,0.5,0.5])
        self.wind_data={}
        self.running = True

   
    def run_atmos(self, rate, truestate):
        while self.running:
            
            # Check if the sensor function is simulate_accelerometer
            wind_data=self.get_wind_data(truestate)
            
            self.wind_data = wind_data
            time.sleep(rate)
        print(f"Atmosphere thread exiting.")

    def get_wind_data(self, state):
        # Unpacking the state vector
        #pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state
        random_wind = np.random.uniform(-1, 1, 3)
        wind_vector = random_wind * self.wind_scale
        return wind_vector

    def start_atmos_thread(self,rate,state):
        thread = threading.Thread(target=self.run_atmos, args=(rate,state,))
        thread.start()
        return thread

    def stop(self):
        self.running = False

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
    
    