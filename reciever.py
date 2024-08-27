#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import zmq
import time
import os

AGENTS_FILE = "agents.txt"

def read_agents_file():
    agents = {}
    if os.path.exists(AGENTS_FILE):
        with open(AGENTS_FILE, "r") as file:
            for line in file:
                agent_id, pub_address, rep_address = line.strip().split(",")
                agents[agent_id] = {"pub_address": pub_address, "rep_address": rep_address}
    return agents

def main():
    context = zmq.Context()

    # Publisher socket to broadcast the list of active agents
    agent_pub_socket = context.socket(zmq.PUB)
    agent_pub_socket.bind("tcp://127.0.0.1:5566")  # Change this if needed

    agents = {}  # Dictionary to store agent info
    agent_sockets = {}  # Dictionary to store ZMQ sockets for each agent

    try:
        while True:
            # Read the current list of agents from the file
            current_agents = read_agents_file()

            # Check for new agents to connect to
            for agent_id, addresses in current_agents.items():
                if agent_id not in agents:
                    pub_address = addresses["pub_address"]
                    rep_address = addresses["rep_address"]
                    print(f"Connecting to new agent {agent_id} at {pub_address}")

                    # Create and connect to the agent's pub socket
                    pub_socket = context.socket(zmq.SUB)
                    pub_socket.connect(pub_address)
                    pub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

                    # Add to the dictionaries
                    agents[agent_id] = {"state": None, "rep_address": rep_address}
                    agent_sockets[agent_id] = pub_socket

            # Check for agents to disconnect from
            for agent_id in list(agents.keys()):
                if agent_id not in current_agents:
                    print(f"Disconnecting from agent {agent_id}")
                    agent_sockets[agent_id].close()
                    del agent_sockets[agent_id]
                    del agents[agent_id]

            # Receive messages from connected agents
            for agent_id, pub_socket in agent_sockets.items():
                try:
                    message = pub_socket.recv_json(flags=zmq.NOBLOCK)
                    state = message["state"]
                    agents[agent_id]["state"] = state
                    print(f"Received state from agent {agent_id}: {state}")
                except zmq.Again:
                    pass

            # Print and publish the list of active agents
            print("Active agents:")
            active_agents = list(agents.keys())
            for agent_id in active_agents:
                print(f"- {agent_id}")

            # Publish the list of active agents
            agent_pub_socket.send_json({"active_agents": active_agents})

            time.sleep(5)

    except KeyboardInterrupt:
        print("Receiver stopping...")
    
    finally:
        for pub_socket in agent_sockets.values():
            pub_socket.close()
        agent_pub_socket.close()
        context.term()

if __name__ == "__main__":
    main()

