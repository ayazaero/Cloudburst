import socket
import threading
from uav import Agent

class AgentManager:
    def __init__(self):
        self.agents = {}
        self.lock = threading.Lock()

    def add_agent(self, agent_id):
        agent = Agent(agent_id)
        self.agents[agent_id] = agent
        threading.Thread(target=agent.run).start()

    def get_agent_state(self, agent_id):
        with self.lock:
            return self.agents[agent_id].state if agent_id in self.agents else None

    def update_agent_waypoint(self, agent_id, new_waypoint):
        with self.lock:
            if agent_id in self.agents:
                self.agents[agent_id].state['waypoint'] = new_waypoint

def start_server(agent_manager):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', 12345))
    server_socket.listen(5)
    print("Server listening on port 12345...")

    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from {addr} established.")
        handle_client(client_socket, agent_manager)

def handle_client(client_socket, agent_manager):
    while True:
        data = client_socket.recv(1024).decode()
        if not data:
            break
        print(f"Received command: {data}")
        process_command(data, agent_manager)
    client_socket.close()

def process_command(command, agent_manager):
    parts = command.split()
    if parts[0] == "ADD_AGENT":
        agent_id = parts[1]
        agent_manager.add_agent(agent_id)
    elif parts[0] == "GET_STATE":
        agent_id = parts[1]
        state = agent_manager.get_agent_state(agent_id)
        print(f"State of {agent_id}: {state}")
    elif parts[0] == "UPDATE_WAYPOINT":
        agent_id = parts[1]
        waypoint = (float(parts[2]), float(parts[3]))
        agent_manager.update_agent_waypoint(agent_id, waypoint)

if __name__ == "__main__":
    agent_manager = AgentManager()
    threading.Thread(target=start_server, args=(agent_manager,)).start()
