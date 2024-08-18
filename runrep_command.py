import zmq
import json

def get_atmos_address():
    with open("environment.txt", "r") as file:
        for line in file:
            agent_id, address = line.strip().split(",")
            return address
    return None

def get_wind_command(command):
    rep_address = get_atmos_address()
    if rep_address is None:
        print("No atmosphere found in environment.txt")
        return
    
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect(rep_address)
    
    message = {"command": command}
    socket.send_json(message)
    
    response = socket.recv_json()
    print("Received reply:", response)
    
    if 'wind_state' in response:
        Vw = response['wind_state']
    else:
        print("No wind state in the response.")
    
    socket.close()
    context.term()

if __name__ == "__main__":
    get_wind_command("get_state")
