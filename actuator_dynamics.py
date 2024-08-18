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
    def __init__(self, name, kp, ki, kd, initial_position, time_constant, min_position, max_position):
        self.name = name
        self.position = initial_position
        self.target_position = initial_position
        self.time_constant = time_constant
        self.min_position = min_position
        self.max_position = max_position
        self.running = True
        self.lock = threading.Lock()
        self.pid = PIDController(kp, ki, kd)
        self.thread = threading.Thread(target=self.update_position)
    
    def set_target_position(self, target):
        # Limit the target position within the defined range
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
            with self.lock:
                control_signal = self.pid.compute(self.target_position, self.position, dt)
                self.position += control_signal * dt / self.time_constant
                # Limit the position within the defined range
                self.position = max(self.min_position, min(self.position, self.max_position))
            previous_time = current_time
            time.sleep(0.01)  # Update at 100 Hz

    def start(self):
        print(f"Starting actuator: {self.name}")
        self.thread.start()
    
    def stop(self):
        self.running = False
        self.thread.join()
        print(f"Stopped actuator: {self.name}")

# Example usage
if __name__ == "__main__":
    # Initialize actuators with PID parameters and limits
    aileron_left = Actuator("aileron_left", kp=1.0, ki=0.1, kd=0.05, initial_position=0.0, time_constant=0.1, min_position=-20.0, max_position=20.0)
    aileron_right = Actuator("aileron_right", kp=1.0, ki=0.1, kd=0.05, initial_position=0.0, time_constant=0.1, min_position=-20.0, max_position=20.0)
    rudder = Actuator("rudder", kp=1.0, ki=0.1, kd=0.05, initial_position=0.0, time_constant=0.1, min_position=-30.0, max_position=30.0)
    elevator = Actuator("elevator", kp=1.0, ki=0.1, kd=0.05, initial_position=0.0, time_constant=0.1, min_position=-10.0, max_position=10.0)
    thrust = Actuator("thrust", kp=1.0, ki=0.1, kd=0.05, initial_position=0.0, time_constant=0.1, min_position=0.0, max_position=2000.0)

    # Set integral windup limits
    aileron_left.pid.set_integral_limits(-10, 10)
    aileron_right.pid.set_integral_limits(-10, 10)
    rudder.pid.set_integral_limits(-10, 10)
    elevator.pid.set_integral_limits(-10, 10)
    thrust.pid.set_integral_limits(-100, 100)  # Different range for thrust

    # Start all actuators
    aileron_left.start()
    aileron_right.start()
    rudder.start()
    elevator.start()
    thrust.start()

    # Set target positions
    aileron_left.set_target_position(10)
    aileron_right.set_target_position(-10)
    rudder.set_target_position(5)
    elevator.set_target_position(3)
    thrust.set_target_position(1000)

    # Main loop to monitor positions
    try:
        while True:
            print("Aileron Left Position:", aileron_left.get_position())
            print("Aileron Right Position:", aileron_right.get_position())
            print("Rudder Position:", rudder.get_position())
            print("Elevator Position:", elevator.get_position())
            print("Thrust Position:", thrust.get_position())
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    # Stop all actuators
    aileron_left.stop()
    aileron_right.stop()
    rudder.stop()
    elevator.stop()
    thrust.stop()
