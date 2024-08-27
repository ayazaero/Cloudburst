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
from atmosphere_dynamics import Atmos

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



class Atmosphere:
    def __init__(self, agent_id, rep_address=None):
        self.agent_id = agent_id
        self.context = zmq.Context()
        self.running=True
        
        if rep_address is None:
            rep_port = get_free_port(5560)
            rep_address = f"tcp://127.0.0.1:{rep_port}"
            print(f"{self.agent_id}: Binding REP socket to {rep_address}")

        self.rep_address = rep_address
        self.rep_socket = self.context.socket(zmq.REP)
        self.rep_socket.bind(self.rep_address)

    def write_address_to_file(self):
        with open("environment.txt", "a") as file:
            file.write(f"{self.agent_id},{self.rep_address}\n")

    def remove_address_from_file(self):
        if os.path.exists("environment.txt"):
            with open("environment.txt", "r") as file:
                lines = file.readlines()
            with open("environment.txt", "w") as file:
                for line in lines:
                    if not line.startswith(self.agent_id):
                        file.write(line)

    def start(self):
        print(f"{self.agent_id}: Starting environment.")

        print(f"{self.agent_id}: Starting atmos threads.")
        self.atmos=Atmos()
        self.write_address_to_file()
        
        # Add a new thread for each sensor
        self.atmos_thread = self.atmos.start_atmos_thread(0.1,[0,0,0])

        print(f"{self.agent_id}: Starting Communication threads.")
        self.rep_thread = threading.Thread(target=self.run_rep)
        self.rep_thread.start()
        
        
 
    def run_rep(self):
        while self.running:
            try:
                message = self.rep_socket.recv_json(flags=zmq.NOBLOCK)  # Non-blocking receive
                command = message.get("command", "")
                if command == "get_state":
                    wind_state = self.atmos.get_wind_data([0,0,0])  # Call the function to get wind state
                    response = {
                        "agent_id": self.agent_id,
                        "wind_state": wind_state.tolist()
                    }
                    self.rep_socket.send_json(response)
                    print(f"{self.agent_id}: Replied with wind state: {wind_state}")
                else:
                    self.rep_socket.send_json({"error": "Unknown command"})
            except zmq.Again:
                pass  # No message received, just continue checking the running flag
            time.sleep(0.001)
        print(f"{self.agent_id}: Rep thread exiting.")



    def stop(self):
        print(f"{self.agent_id}: Stopping agent.")
        self.running = False
        self.atmos.stop()
        
        
        self.rep_thread.join()
        
        self.atmos_thread.join()
       
        self.rep_socket.close()
        self.context.term()

        self.remove_address_from_file()
        print(f"{self.agent_id}: Removed data from file.")


if __name__ == "__main__":
    atmos_id = str(uuid.uuid4())
    atmos=Atmosphere(atmos_id)
    atmos.start()
    time.sleep(1.0)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        atmos.stop()
