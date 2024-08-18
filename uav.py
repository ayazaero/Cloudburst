import threading
import time
import zmq
import uuid
import numpy as np
import os
from sensor_run import Sensors
from internal_dynamics import InternalDynamics
from actuator_dynamics import Actuator

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


class HighLevelController:
    def __init__(self, dynamics):
        self.dynamics = dynamics
        self.target_position = np.array([10.0, 10.0, 10.0])

    def update(self):
        error = self.target_position - self.dynamics.state[0:3]
        control_inputs = error * 0.1  # Proportional gain
        return control_inputs

class LowLevelController:
    def __init__(self, dynamics):
        self.dynamics = dynamics

    def update(self, control_inputs):
        self.dynamics.update(control_inputs)

class UAVAgent:
    def __init__(self, agent_id,launchpoint, pub_address=None, rep_address=None, sense_address=None):
        self.agent_id = agent_id
        self.sensor_data = {}
        self.context = zmq.Context()
        self.ref_lla = launchpoint

        if pub_address is None:
            pub_port = get_free_port(5557)
            pub_address = f"tcp://127.0.0.1:{pub_port}"
            print(f"{self.agent_id}: Binding PUB socket to {pub_address}")
        self.pub_address = pub_address
        self.running = True
        self.pub_socket = self.context.socket(zmq.PUB)
        self.pub_socket.bind(self.pub_address)

        if rep_address is None:
            rep_port = get_free_port(5560)
            rep_address = f"tcp://127.0.0.1:{rep_port}"
            print(f"{self.agent_id}: Binding REP socket to {rep_address}")

        self.rep_address = rep_address
        self.rep_socket = self.context.socket(zmq.REP)
        self.rep_socket.bind(self.rep_address)

        



        self.dynamics = InternalDynamics(1.0,1.0)
        self.high_level_controller = HighLevelController(self.dynamics)
        self.low_level_controller = LowLevelController(self.dynamics)

        print(f"{self.agent_id}: Initialized UAV agent.")
        
        self.add_agent_to_file()

    def start(self):
        print(f"{self.agent_id}: Starting agent threads.")

        print(f"{self.agent_id}: Initializing Sensors.")
        self.sensor=Sensors(agent_id)

        time.sleep(1)
        print(f"{self.agent_id}: Sensor Initialized")
        print(f"{self.agent_id}: Starting sensor threads.")
        
        # Add a new thread for each sensor
        self.accel_thread = self.sensor.start_sensor_thread(self.sensor.simulate_accelerometer,0.1,self.dynamics,self.ref_lla,False)
        self.gyro_thread = self.sensor.start_sensor_thread(self.sensor.simulate_gyro,0.1,self.dynamics,self.ref_lla,False)
        self.altimeter_thread = self.sensor.start_sensor_thread(self.sensor.simulate_altimeter,0.5,self.dynamics,self.ref_lla,False)
        self.pitot_thread = self.sensor.start_sensor_thread(self.sensor.simulate_pitot_tube,0.5,self.dynamics,self.ref_lla,False)
        self.magnetometer_thread = self.sensor.start_sensor_thread(self.sensor.simulate_magnetometer,0.2,self.dynamics,self.ref_lla,False)
        self.gps_thread = self.sensor.start_sensor_thread(self.sensor.simulate_gps,1.0,self.dynamics,self.ref_lla,True)

        print(f"{self.agent_id}: Starting controller threads.")
        self.control_thread = threading.Thread(target=self.run_control)
        self.control_thread.start()

        print(f"{self.agent_id}: Starting Communication threads.")
        self.pub_thread = threading.Thread(target=self.run_pub)
        self.pub_thread.start()
        self.rep_thread = threading.Thread(target=self.run_rep)
        self.rep_thread.start()
        
        
 
    def run_pub(self):
        while self.running:
            time.sleep(1)
            state = self.dynamics.get_state()
            message = {
                "agent_id": self.agent_id,
                "state": state.tolist()
            }
            self.pub_socket.send_json(message)
            #print(f"{self.agent_id}: Published state: {state}")
            #print(self.sensor.sensor_data["simulate_magnetometer"],self.sensor.sensor_data["simulate_gps"])
            
        print(f"{self.agent_id}: Pub thread exiting.")

    

    def run_rep(self):
        while self.running:
            try:
                message = self.rep_socket.recv_json(flags=zmq.NOBLOCK)  # Non-blocking receive
                command = message.get("command", "")
                if command == "get_state":
                    state = self.dynamics.get_state()
                    response = {
                        "agent_id": self.agent_id,
                        "state": state.tolist()
                    }
                    self.rep_socket.send_json(response)
                    print(f"{self.agent_id}: Replied with state: {state}")
                else:
                    self.rep_socket.send_json({"error": "Unknown command"})
            except zmq.Again:
                pass  # No message received, just continue checking the running flag
            time.sleep(0.1)
        print(f"{self.agent_id}: Rep thread exiting.")

    def run_control(self):
        while self.running:
            control_inputs = self.high_level_controller.update()
            self.low_level_controller.update(control_inputs)
            #print(f"{self.agent_id}: Updated control inputs: {control_inputs}")
            time.sleep(0.1)
        print(f"{self.agent_id}: Control thread exiting.")

    def stop(self):
        print(f"{self.agent_id}: Stopping agent.")
        self.running = False
        self.sensor.stop()
        
        
        self.pub_thread.join()
        self.rep_thread.join()
        self.control_thread.join()
        
        self.accel_thread.join()
        self.gyro_thread.join()
        self.altimeter_thread.join()
        self.pitot_thread.join()
        self.magnetometer_thread.join()
        self.gps_thread.join()

        self.pub_socket.close()
        self.rep_socket.close()
        self.context.term()

        print(f"{self.agent_id}: Agent stopped.")
        # Remove agent details from file
        self.remove_agent_from_file()
        print(f"{self.agent_id}: Agent removed from file.")

    def add_agent_to_file(self):
        with open("agents.txt", "a") as file:
            file.write(f"{self.agent_id},{self.pub_address},{self.rep_address}\n")

    def remove_agent_from_file(self):
        if not os.path.exists("agents.txt"):
            return
        with open("agents.txt", "r") as file:
            lines = file.readlines()
        with open("agents.txt", "w") as file:
            for line in lines:
                if not line.startswith(self.agent_id):
                    file.write(line)

if __name__ == "__main__":
    agent_id = str(uuid.uuid4())
    ref_lla=[47.653784,-122.307814,1.0]
    agent = UAVAgent(agent_id,ref_lla)
    agent.start()
    time.sleep(1.0)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        agent.stop()
