#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import threading
import time

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_min = float('-inf')
        self.integral_max = float('inf')
    
    def set_integral_limits(self, integral_min, integral_max):
        self.integral_min = integral_min
        self.integral_max = integral_max

    def compute(self, setpoint, measurement, dt):
        if dt == 0:
            return 0  # Avoid division by zero
        
        error = setpoint - measurement
        self.integral += error * dt
        # Prevent integral windup
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


class Actuator:
    def __init__(self, name, kp, ki, kd, initial_position, natural_frequency, damping_ratio, min_position, max_position):
        self.name = name
        self.position = initial_position
        self.velocity = 0.0
        self.acceleration = 0.0
        self.target_position = initial_position
        self.min_position = min_position
        self.max_position = max_position
        self.running = True
        self.lock = threading.Lock()
        self.pid = PIDController(kp, ki, kd)
        self.natural_frequency = natural_frequency
        self.damping_ratio = damping_ratio
        self.omega_squared = natural_frequency**2  # ω²
        self.two_zeta_omega = 2 * damping_ratio * natural_frequency  # 2ζω
        self.thread = threading.Thread(target=self.update_position)

    def set_target_position(self, target):
        with self.lock:
            self.target_position = max(self.min_position, min(target, self.max_position))
    
    def get_position(self):
        with self.lock:
            return self.position
    
    def update_position(self):
        previous_time = time.time()
        while self.running:
            current_time = time.time()
            dt = current_time - previous_time
            if dt > 0:
                with self.lock:
                    control_signal = self.pid.compute(self.target_position, self.position, dt)
                    # Update acceleration based on the PID output and current velocity
                    self.acceleration = self.omega_squared * (control_signal - self.position) - self.two_zeta_omega * self.velocity
                    # Update velocity and position using Euler integration
                    self.velocity += self.acceleration * dt
                    self.position += self.velocity * dt
                    # Limit the position within the defined range
                    self.position = max(self.min_position, min(self.position, self.max_position))
                    
            previous_time = current_time
            time.sleep(0.005)  # Update at 200 Hz

    def start(self):
        print(f"Starting actuator: {self.name}")
        self.thread.start()
    
    def stop(self):
        self.running = False
        self.thread.join()
        print(f"Stopped actuator: {self.name}")
