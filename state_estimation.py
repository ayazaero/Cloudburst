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

class state_estimator:
    def __init__(self, agent_id):
        self.agent_id=agent_id
        self.subscribe_to_agent()

    def find_agent_port(self, filename="agents_sensors.txt"):
        with open(filename, 'r') as file:
            lines = file.readlines()
            for line in lines:
                if line.startswith(self.agent_id):
                    return line.split(',')[1].strip()
        return None

    def on_data_received(self,data):
        print(f"Data received: {data}")
        # Execute the function with the received data

    def subscribe_to_agent(self):
        # Find the agent's port
        agent_port = self.find_agent_port()
        if not agent_port:
            print(f"Agent ID {self.agent_id} not found.")
            return
        
        # Prepare the context and subscriber
        context = zmq.Context()
        socket = context.socket(zmq.SUB)
        socket.connect(agent_port)
        socket.setsockopt_string(zmq.SUBSCRIBE, "")
        
        print(f"Subscribed to agent {self.agent_id} at {agent_port}")
        
        while True:
            data = socket.recv_string()
            self.on_data_received(data)

    
if __name__=="__main__":
    agent_id="d7ed5d09-72d4-4ae5-be25-22c2e2e41174"
    statest=state_estimator(agent_id)
    statest.subscribe_to_agent()