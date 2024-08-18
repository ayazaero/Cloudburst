import numpy as np
import uav_paramters as up
import zmq

class InternalDynamics:
    def __init__(self, mass,payload_mass):
        # State vector:
        # [pn, pe, pd, u, v, w, φ (phi), θ (theta), ψ (psi), p, q, r]
        self.state = np.array([[0.0], [0], [0], [1.0], [0], [0], [0.0], [0.0], [90.0], [0], [0], [0]])
        self.mass=mass
        self.Vw=self.get_wind_command("get_state")
        self.payload_mass = payload_mass
        self.total_mass=mass+payload_mass
        self.sdot = np.array([[0.0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

    def update(self, control_inputs, dt=0.1):
        # Assuming control_inputs contain [force_x, force_y, force_z, torque_x, torque_y, torque_z]
        force = self.get_force(self.total_mass,control_inputs)
        moments = self.get_moments()

        # Extract state variables
        pn, pe, pd, u, v, w, phi, tht, psi, p, q, r = self.state.flatten()
        phi=np.radians(phi)
        tht=np.radians(tht)
        psi=np.radians(psi)

        # Rotation matrix for the body frame to inertial frame
        R = np.array([
            [np.cos(tht) * np.cos(psi), (np.sin(phi) * np.sin(tht) * np.cos(psi)) - (np.cos(phi) * np.sin(psi)), (np.cos(phi) * np.sin(tht) * np.cos(psi)) + (np.sin(phi) * np.sin(psi))],
            [np.cos(tht) * np.sin(psi), (np.sin(phi) * np.sin(tht) * np.sin(psi)) + (np.cos(phi) * np.cos(psi)), (np.cos(phi) * np.sin(tht) * np.sin(psi)) - (np.sin(phi) * np.cos(psi))],
            [-np.sin(tht), np.sin(phi) * np.cos(tht), np.cos(phi) * np.cos(tht)]
        ])

        posdot = R @ np.array([[u], [v], [w]])
        udot = np.array([[(r * v) - (q * w)], [(p * w) - (r * u)], [(q * u) - (p * v)]]) + (force / (self.payload_mass+self.mass))

        # Rotation matrix for angular velocities
        R1 = np.array([
            [1.0, np.sin(phi) * np.tan(tht), np.cos(phi) * np.tan(tht)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi) / np.cos(tht), np.cos(phi) / np.cos(tht)]
        ])

        phidot = R1 @ np.array([[p], [q], [r]])

        # Moment of inertia parameters
        J_x = up.J_x
        J_y = up.J_y
        J_z = up.J_z
        J_xz = up.J_xz

        C = (J_x * J_z) - J_xz**2
        C1 = (J_xz * (J_x - J_y + J_z)) / C
        C2 = (J_z * (J_z - J_y) + J_xz**2) / C
        C3 = J_z / C
        C4 = J_xz / C
        C5 = (J_z - J_x) / J_y
        C6 = (J_xz / J_y)
        C7 = ((J_x - J_y) * J_x + J_xz**2) / C
        C8 = J_x / C

        pdot = (C1 * p * q) - (C2 * q * r) + (C3 * moments[0]) + (C4 * moments[2])
        qdot = (C5 * p * r) - (C6 * (p**2 - r**2)) + (moments[1] / J_y)
        rdot = (C7 * p * q) - (C1 * q * r) + (C4 * moments[0]) + (C8 * moments[2])

        # Combine state derivatives
        self.sdot = np.vstack((posdot, udot, phidot, pdot, qdot, rdot))

        # Update state
        self.state += self.sdot * dt

    def get_state(self):
        return self.state
    
    def get_force(self,mass,control_inputs):
        f=np.zeros((3, 1))
        #f[1]=-9.8*(self.payload_mass+self.mass)
        return f
    
    def get_moments(self):
        return np.zeros((3, 1))
    
    def get_atmos_address(self):
        with open("environment.txt", "r") as file:
            for line in file:
                agent_id, address = line.strip().split(",")
                return address
        return None

    def get_wind_command(self,command):
        rep_address = self.get_atmos_address()
        if rep_address is None:
            print("No atmosphere found in environment.txt")
            return
        
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        socket.connect(rep_address)
        
        message = {"command": command}
        socket.send_json(message)
        
        response = socket.recv_json()
        
        if 'wind_state' in response:
            Vw = response['wind_state']
        else:
            print("No wind state in the response.")
        
        socket.close()
        context.term()

        return Vw
