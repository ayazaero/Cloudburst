#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import threading
import time
import zmq
import uuid
import numpy as np
import os
import json


from sensor_run import Sensors
from internal_dynamics import InternalDynamics
from actuator_dynamics import Actuator
import uav_paramters as up
from agent_controller import Controller


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

        # Initialize actuators with PID parameters and limits
        self.aileron_left = Actuator("aileron_left", kp=70.0, ki=15, kd=0.5,initial_position=0.0,natural_frequency=10,damping_ratio=5,min_position=-up.max_delta_a_l,max_position=up.max_delta_a_l)
        self.aileron_right = Actuator("aileron_right", kp=70.0, ki=15, kd=0.5,initial_position=0.0,natural_frequency=10,damping_ratio=5,min_position=-up.max_delta_a_r,max_position=up.max_delta_a_r)
        self.rudder = Actuator("rudder", kp=70.0, ki=15, kd=0.5,initial_position=0.0,natural_frequency=10,damping_ratio=5,min_position=-up.max_delta_r,max_position=up.max_delta_r)
        self.elevator = Actuator("elevator", kp=70.0, ki=15, kd=0.5,initial_position=-4.4460*np.pi/180,natural_frequency=10,damping_ratio=5,min_position=-up.max_delta_e,max_position=up.max_delta_e)
        self.thrust = Actuator("thrust_pwm", kp=50.0, ki=30, kd=0.5,initial_position=0.3986,natural_frequency=10,damping_ratio=5,min_position=0,max_position=up.max_delta_t)

        # Set integral windup limits
        self.aileron_left.pid.set_integral_limits(-10*np.pi/180, 10*np.pi/180)
        self.aileron_right.pid.set_integral_limits(-10*np.pi/180, 10*np.pi/180)
        self.rudder.pid.set_integral_limits(-10*np.pi/180, 10*np.pi/180)
        self.elevator.pid.set_integral_limits(-10*np.pi/180, 10*np.pi/180)
        self.thrust.pid.set_integral_limits(-2, 2)  # Different range for thrust
        
        self.elevator.set_target_position(-4.4460*np.pi/180)
        self.thrust.set_target_position(0.3986)

        self.dynamics = InternalDynamics(up.m,0.0,self.aileron_left,self.aileron_right,self.rudder,self.elevator,self.thrust)
        self.controller = Controller(self.dynamics,self.aileron_left,self.aileron_right,self.rudder,self.elevator,self.thrust)

        
        #self.high_level_controller = HighLevelController(self.dynamics)
        #self.low_level_controller = LowLevelController(self.dynamics)

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

        print(f"{self.agent_id}: Starting Actuators.")
        

        # Start all actuators
        self.aileron_left.start()
        self.aileron_right.start()
        self.rudder.start()
        self.elevator.start()
        self.thrust.start()
        time.sleep(1)
        print(f"{self.agent_id}: Actuators running.")


        

        
        print(f"{self.agent_id}: Starting Dynamics threads.")
        self.dynamics.start()
        #self.dynamics.controller_running=1
        time.sleep(5)
        print(f"{self.agent_id}: Starting controller threads.")
        self.controller.start()
        
        time.sleep(1)

        print(f"{self.agent_id}: Starting Communication threads.")
        self.pub_thread = threading.Thread(target=self.run_pub)
        self.pub_thread.start()
        self.rep_thread = threading.Thread(target=self.run_rep)
        self.rep_thread.start()
        
        
 
    def run_pub(self):
        while self.running:
            time.sleep(0.1)
            state = self.dynamics.get_state()
            state=state.flatten()
            if hasattr(state, 'tolist'):
                state = state.tolist()
            

            message = {
                "agent_id": self.agent_id,
                "state": state
            }
            #json_message = json.dumps(message)
            self.pub_socket.send_json(message)
            #print(f"Published: {message}")
            #print(self.sensor.sensor_data) Sensor is not saving data anymore, it is directly publishing to the port
            
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
            time.sleep(0.01)
        print(f"{self.agent_id}: Control thread exiting.")

    def stop(self):
        print(f"{self.agent_id}: Stopping agent.")
        self.running = False
        self.sensor.stop()
        
        
        self.pub_thread.join()
        self.rep_thread.join()
        
        
        self.accel_thread.join()
        self.gyro_thread.join()
        self.altimeter_thread.join()
        self.pitot_thread.join()
        self.magnetometer_thread.join()
        self.gps_thread.join()

        # Stop all actuators
        self.aileron_left.stop()
        self.aileron_right.stop()
        self.rudder.stop()
        self.elevator.stop()
        self.thrust.stop()

        self.dynamics.stop()
        
        time.sleep(0.5)
        self.controller.stop()
        
        

        self.pub_socket.close()
        self.rep_socket.close()
        self.dynamics.windsocket.close()


        self.dynamics.windcontext.term()
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
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        agent.stop()
