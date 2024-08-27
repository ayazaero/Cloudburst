#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import zmq
import time
import numpy as np
import matplotlib.pyplot as plt
import threading
import os
from queue import Queue

AGENTS_FILE = "agents.txt"

def read_agents_file():
    agents = {}
    if os.path.exists(AGENTS_FILE):
        with open(AGENTS_FILE, "r") as file:
            for line in file:
                agent_id, pub_address, rep_address = line.strip().split(",")
                agents[agent_id] = {"pub_address": pub_address, "rep_address": rep_address}
    return agents

def get_agent_positions(agents):
    positions = {}
    for agent_id, addresses in agents.items():
        try:
            context = zmq.Context()
            rep_socket = context.socket(zmq.REQ)
            rep_socket.connect(addresses["rep_address"])
            # Request the agent's position
            request = {"command": "get_data"}
            rep_socket.send_json(request)
            response = rep_socket.recv_json()
            positions[agent_id] = response["data"]["position"]
            rep_socket.close()
        except Exception as e:
            print(f"Error getting data from agent {agent_id}: {e}")
    return positions

def plot_positions(positions, ax, colors):
    ax.cla()  # Clear the current axes
    if not positions:
        print("No positions to plot.")
        return

    # Use predefined colors for consistency
    for i, (agent_id, position) in enumerate(positions.items()):
        ax.scatter(position[0], position[1], position[2], color=colors[i], label=agent_id)

    # Set fixed axis limits
    ax.set_xlim([-200, 200])  # Adjust based on your expected range
    ax.set_ylim([-200, 200])
    ax.set_zlim([-200, 200])

    ax.set_xlabel('X Position')
    ax.set_ylabel('Y Position')
    ax.set_zlabel('Z Position')
 
    ax.legend()
    plt.draw()  # Explicitly draw the plot

def agent_monitor(stop_event, queue):
    while not stop_event.is_set():
        agents = read_agents_file()
        if not agents:
            print("No agents found. Waiting...")
            time.sleep(10)  # Wait before trying again
            continue

        positions = get_agent_positions(agents)
        queue.put(positions)  # Pass positions to the main thread

        time.sleep(1)  # Wait before checking again

if __name__ == "__main__":
    stop_event = threading.Event()
    position_queue = Queue()
    monitor_thread = threading.Thread(target=agent_monitor, args=(stop_event, position_queue))
    monitor_thread.start()

    plt.ion()  # Enable interactive mode
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Predefined colors for the agents
    colors = plt.cm.viridis(np.linspace(0, 1, 10))  # Adjust number of colors based on expected agents

    try:
        while monitor_thread.is_alive():
            if not position_queue.empty():
                positions = position_queue.get()
                plot_positions(positions, ax, colors)
                plt.pause(0.1)  # Pause to allow the plot to update
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping the monitor...")
        stop_event.set()  # Signal the monitor to stop
        monitor_thread.join()  # Wait for the thread to finish

    plt.ioff()  # Disable interactive mode
    plt.show()  # Show the plot (optional, to keep it open after stopping)
    print("Monitor stopped.")
