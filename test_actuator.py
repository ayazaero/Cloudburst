import time
import numpy as np
import matplotlib.pyplot as plt
from actuator_dynamics import Actuator

target = np.array([1000, 0, 1000, 1500, 2000, 2500, 3000, 2000, 1000, 0])
target = target/300.0

def main():
    # Initialize actuator with PID parameters
    #thrust = Actuator("thrust", kp=5.0, ki=0.5, kd=0.05,initial_position=0.0,time_constant=0.1,min_position=0.0,max_position=3000.0)
    thrust = Actuator("aileron_left", kp=5.0, ki=0.5, kd=0.05,initial_position=0.0,time_constant=0.1,min_position=-12.0,max_position=12.0)
    thrust.pid.set_integral_limits(-100, 100)  # Different range for thrust

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
                actual_position = thrust.get_position()
                elapsed_time = time.time() - start_time

                # Store the values for plotting
                times.append(elapsed_time)
                target_positions.append(t)
                actual_positions.append(actual_position)

                # Print the current thrust position
                print(f"Thrust Position: {actual_position:.2f}")


                time.sleep(0.1)  # Monitor every 0.1 seconds

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
