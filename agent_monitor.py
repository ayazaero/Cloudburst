#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import tkinter as tk
from tkinter import ttk, font
import zmq
import json
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D
import threading
import time

def read_agents(filename):
    agents = {}
    try:
        with open(filename, 'r') as file:
            for line in file:
                agent_id, pub_addr, _ = line.strip().split(',')
                agents[agent_id] = pub_addr
    except FileNotFoundError:
        print("File not found.")
    return agents

def update_agents(app, filename):
    while True:
        current_agents = read_agents(filename)
        app.update_dropdown(current_agents)
        time.sleep(1)

class App:
    def __init__(self, root, filename):
        self.root = root
        self.filename = filename
        self.agents = read_agents(filename)
        self.current_agent = None
        self.subscriber = None
        self.context = zmq.Context()
        self.data_buffer = {'height': [], 'velocity': [], 'phi': [], 'tht': [], 'psi': [], 'x': [], 'y': [], 'z': []}
        self.is_paused = False

        self.default_font = font.nametofont("TkDefaultFont")
        self.default_font.configure(size=12)
        self.text_font = font.Font(family="Helvetica", size=12)
        root.option_add("*Font", self.default_font)
        root.geometry("2000x1100")

        self.dropdown = ttk.Combobox(root, values=list(self.agents.keys()))
        self.dropdown.grid(row=0, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        self.dropdown.bind("<<ComboboxSelected>>", self.on_agent_selected)

        self.pause_button = tk.Button(root, text="Pause", command=self.toggle_pause)
        self.pause_button.grid(row=1, column=0, padx=10, pady=10)

        self.close_button = tk.Button(root, text="Close", command=self.close_app)
        self.close_button.grid(row=1, column=1, padx=10, pady=10)
        
        self.block1 = tk.Text(root, height=30, width=50, font=self.text_font)
        self.block1.grid(row=2, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        
        self.block2 = tk.Text(root, height=20, width=50, font=self.text_font)
        self.block2.grid(row=3, column=0, columnspan=2, padx=10, pady=10, sticky="ew")
        
        # 2D plots
        self.figure, self.ax = plt.subplots(3, 1, figsize=(7, 10))
        self.canvas = FigureCanvasTkAgg(self.figure, master=root)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=2, rowspan=4, padx=10, pady=10, sticky="nsew")

        # 3D plot
        self.figure3d = Figure(figsize=(7, 10))
        self.ax3d = self.figure3d.add_subplot(111, projection='3d')
        self.canvas3d = FigureCanvasTkAgg(self.figure3d, master=root)
        self.canvas3d.draw()
        self.canvas3d.get_tk_widget().grid(row=0, column=3, rowspan=4, padx=10, pady=10, sticky="nsew")

        if self.agents:
            self.dropdown.set(list(self.agents.keys())[-1])  # Set to last agent in the list
            self.on_agent_selected(None)  # Connect to the last agent automatically

    def on_agent_selected(self, event):
        new_agent = self.dropdown.get()
        if new_agent != self.current_agent:
            self.disconnect_current_agent()
            self.data_buffer = {'height': [], 'velocity': [], 'phi': [], 'tht': [], 'psi': [], 'x': [], 'y': [], 'z': []}
            self.connect_to_agent(new_agent)

    # Other methods remain unchanged

    def update_plot_data(self, state):
        self.data_buffer['height'].append(-state[2])  # Assuming state[2] is depth/altitude
        self.data_buffer['velocity'].append(np.linalg.norm(state[3:6]))
        self.data_buffer['phi'].append(state[6])
        self.data_buffer['tht'].append(state[7])
        self.data_buffer['psi'].append(state[8])
        self.data_buffer['x'].append(state[0])  # Assuming state[0] is x-coordinate
        self.data_buffer['y'].append(state[1])  # Assuming state[1] is y-coordinate
        self.data_buffer['z'].append(-state[2])  # Assuming state[2] is z-coordinate
        self.plot_data()

    def plot_data(self):
        # Update 2D plots
        self.ax[0].cla()
        self.ax[1].cla()
        self.ax[2].cla()
        
        self.ax[0].plot(self.data_buffer['height'], label="Height", marker='.')
        self.ax[1].plot(self.data_buffer['velocity'], label="Velocity", marker='.')
        self.ax[2].plot(self.data_buffer['phi'], label="Phi", marker='.')
        self.ax[2].plot(self.data_buffer['tht'], label="Theta", marker='.')
        self.ax[2].plot(self.data_buffer['psi'], label="Psi", marker='.')
        
        self.ax[0].legend()
        self.ax[1].legend()
        self.ax[2].legend()
        self.canvas.draw()

        # Update 3D plot
        self.ax3d.clear()
        self.ax3d.plot3D(self.data_buffer['x'], self.data_buffer['y'], self.data_buffer['z'], label='Trajectory', marker='.')
        self.ax3d.set_xlabel('X')
        self.ax3d.set_ylabel('Y')
        self.ax3d.set_zlabel('Z')
        self.ax3d.legend()
        self.canvas3d.draw()

    def update_dropdown(self, new_agents):
        if new_agents != self.agents:
            self.agents = new_agents
            self.dropdown['values'] = list(self.agents.keys())
            if self.current_agent not in self.agents:
                self.disconnect_current_agent()

    

    def connect_to_agent(self, agent_id):
        if self.subscriber:
            self.subscriber.close()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect(self.agents[agent_id])
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        self.current_agent = agent_id
        self.update_data()

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        self.pause_button.config(text="Resume" if self.is_paused else "Pause")
        if not self.is_paused:
            self.data_buffer = {'height': [], 'velocity': [], 'phi': [], 'tht': [], 'psi': []}

    def disconnect_current_agent(self):
        if self.subscriber:
            self.subscriber.close()
            self.subscriber = None
        self.current_agent = None
        self.data_buffer = {'height': [], 'velocity': [], 'phi': [], 'tht': [], 'psi': []}

    def close_app(self):
        self.disconnect_current_agent()
        self.context.term()
        self.root.destroy()

    def update_data(self):
        if self.current_agent and not self.is_paused:
            try:
                message = self.subscriber.recv_string(flags=zmq.NOBLOCK)
                data = json.loads(message)
                state = data['state']
                self.display_state(state)
                self.update_plot_data(state)
            except zmq.Again:
                pass
        self.root.after(50, self.update_data)

    def display_state(self, state):
        formatted_state = "\n".join([
            f"Position N: {state[0]}",
            f"Position E: {state[1]}",
            f"Position D: {state[2]}",
            f"Velocity u: {state[3]}",
            f"Velocity v: {state[4]}",
            f"Velocity w: {state[5]}",
            f"Phi: {state[6]}",
            f"Theta: {state[7]}",
            f"Psi: {state[8]}",
            f"Angular Velocity p: {state[9]}",
            f"Angular Velocity q: {state[10]}",
            f"Angular Velocity r: {state[11]}"
        ])
        self.block1.delete('1.0', tk.END)
        self.block1.insert(tk.END, formatted_state)

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root, "agents.txt")
    agent_thread = threading.Thread(target=update_agents, args=(app, "agents.txt"), daemon=True)
    agent_thread.start()
    root.mainloop()
