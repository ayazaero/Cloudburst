import time
import numpy as np
import matplotlib.pyplot as plt
from actuator_dynamics import Actuator
import uav_paramters as up

target = np.array([0, 200, 400, 600, 800, 1000, 500,0])
target = target/2000

def main():
    # Initialize actuator with PID parameters
    thrust = Actuator("elevator", kp=70.0, ki=15, kd=0.5,initial_position=0.0,natural_frequency=10,damping_ratio=5,min_position=-up.max_delta_e,max_position=up.max_delta_e)
    #thrust = Actuator("aileron_left", kp=30.0, ki=15, kd=0.5,initial_position=0.0,natural_frequency=10,damping_ratio=0.6,min_position=-12.0,max_position=12.0)
    thrust.pid.set_integral_limits(-1, 1)  # Different range for thrust

    thrust.start()

    # Arrays to store time, target position, and actual position for plotting
    times = []
    target_positions = []
    actual_positions = []

    start_time = time.time()
    try:
        for t in target:
            thrust.set_target_position(t)
            current_time = time.time()
            end_time = current_time + 2  # Change target every 2 seconds
            while time.time() < end_time:
                
                elapsed_time = time.time() - start_time

                # Store the values for plotting
                times.append(elapsed_time)
                target_positions.append(t)
                actual_positions.append(thrust.get_position())

                # Print the current thrust position
                print(f"Thrust Position: {t:.2f}")


                time.sleep(0.01)  # Monitor every 0.1 seconds

    except KeyboardInterrupt:
        pass

    finally:
        thrust.stop()

    # Plot target vs. actual positions over time
    plt.figure()
    plt.plot(times, target_positions, label='Target Position', linestyle='--')
    plt.plot(times, actual_positions, label='Actual Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position')
    plt.legend()
    plt.title('Target vs. Actual Position Over Time')
    plt.show()

if __name__ == "__main__":
    main()
