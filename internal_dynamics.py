#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import numpy as np
import uav_paramters as up
import zmq
import time
import threading
from scipy.integrate import odeint

np.set_printoptions(linewidth=np.inf)

class InternalDynamics:
    def __init__(self, mass,payload_mass,aileron_l,aileron_r,rudder,elevator,thruster):
        self.running =True
        self.lock = threading.Lock()
        
        # State vector:
        # [pn, pe, pd, u, v, w, φ (phi), θ (theta), ψ (psi), p, q, r]
        self.state = np.array([[0.0], [0], [-100], [29.9753], [0.00], [1.2169], [0.0], [2.3249], [0.0], [0], [0], [0]])
        #self.state=initCondition.flatten()
        self.mass=mass
        
        self.aileron_l = aileron_l
        self.aileron_r = aileron_r
        self.rudder = rudder
        self.elevator = elevator
        self.thruster = thruster

        self.payload_mass = payload_mass
        self.total_mass=mass+payload_mass
        self.sdot = np.array([[0.0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0], [0]])

        ## for wind data
        rep_address = self.get_atmos_address()
        if rep_address is None:
            print("No atmosphere found in environment.txt")
            return
        
        self.windcontext = zmq.Context()
        self.windsocket = self.windcontext.socket(zmq.REQ)
        self.windsocket.connect(rep_address)

        self.Vwi=self.get_wind_command("get_state") #Wind speed in inertial frame
        self.timenow=time.time()
        self.Va=np.linalg.norm(self.state[6:9])
        self.alpha=0
        self.beta=0

        self.thread = threading.Thread(target=self.run_dynamics)
        self.controller_running=0

    def run_dynamics(self):
        previous_time = time.time()
        while self.running:
            loop_start_time = time.time()  # Start time of the loop

            current_time = time.time()
            dt = current_time - previous_time
            if dt > 0:
                with self.lock:
                    self.compute()
                    self.state += self.sdot * dt
                    #print(f"states: ,{self.state.T}")

            previous_time = current_time

            loop_end_time = time.time()  # End time of the loop
            loop_duration = loop_end_time - loop_start_time  # Calculate duration of the loop
            #print(f"Loop duration: {loop_duration:.6f} seconds")  # Print the duration of the loop

            time.sleep(0.01)  # Update at 100 Hz
    
    '''def run_dynamics(self):
        previous_time = time.time()
        while self.running:
            current_time = time.time()
            dt = current_time - previous_time
            if dt > 0.005:  # Minimum step size to avoid too frequent updates
                with self.lock:
                    t_span = [0, dt]
                    y0=self.state
                    self.state = odeint(self.compute, y0, t_span, args=(self,))[-1]
                previous_time = current_time
            time.sleep(0.01)  # Adjust the frequency of updates as needed (e.g., for 100 Hz, sleep for 0.01 seconds)'''


    def compute(self):  #,state,t,obj
        # Assuming control_inputs contain [force_x, force_y, force_z, torque_x, torque_y, torque_z]
        self.Vwi=self.get_wind_command("get_state") #Wind speed in inertial frame
        
        # Extract state variables
        pn, pe, pd, u, v, w, phi, tht, psi, p, q, r = map(np.float64, self.state)
        
        phi=np.radians(phi)
        tht=np.radians(tht)
        psi=np.radians(psi)

        #print(phi)
        # Rotation matrix for the body frame to inertial frame
        R = np.array([
            [np.cos(tht) * np.cos(psi), (np.sin(phi) * np.sin(tht) * np.cos(psi)) - (np.cos(phi) * np.sin(psi)), (np.cos(phi) * np.sin(tht) * np.cos(psi)) + (np.sin(phi) * np.sin(psi))],
            [np.cos(tht) * np.sin(psi), (np.sin(phi) * np.sin(tht) * np.sin(psi)) + (np.cos(phi) * np.cos(psi)), (np.cos(phi) * np.sin(tht) * np.sin(psi)) - (np.sin(phi) * np.cos(psi))],
            [-np.sin(tht), np.sin(phi) * np.cos(tht), np.cos(phi) * np.cos(tht)]
        ])

        force,moments = self.get_force_moments(self.total_mass,R)
        
        
        posdot = R @ (np.array([[u], [v], [w]]))
        
        #print('force: ',force)
        udot = np.array([[(r * v) - (q * w)], [(p * w) - (r * u)], [(q * u) - (p * v)]]) + (force / (self.payload_mass+self.mass))
        
        # Rotation matrix for angular velocities
        R1 = np.array([
            [1.0, np.sin(phi) * np.tan(tht), np.cos(phi) * np.tan(tht)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi) / np.cos(tht), np.cos(phi) / np.cos(tht)]
        ])

        phidot = R1 @ np.array([[p], [q], [r]])

        #print(phidot)

        pdot = (up.C1 * p * q) - (up.C2 * q * r) + (up.C3 * moments[0]) + (up.C4 * moments[2])
        qdot = (up.C5 * p * r) - (up.C6 * (p**2 - r**2)) + (moments[1] / up.J_y)
        rdot = (up.C7 * p * q) - (up.C1 * q * r) + (up.C4 * moments[0]) + (up.C8 * moments[2])

        # Combine state derivatives
        #print(posdot.shape)
        #print(udot.shape)
        #print(phidot.shape)

        self.sdot = self.controller_running*np.vstack((posdot, udot, phidot*180/np.pi, pdot, qdot, rdot))
        
        #self.sdot = np.vstack((posdot, udot, phidot, pdot, qdot, rdot))
        #print(self.sdot.shape)
        #return sdot.flatten()
    
    def get_state(self):
        return self.state
    
    def get_force_moments(self,mass,R):
        pn, pe, pd, u, v, w, phi, tht, psi, p, q, r = map(np.float64, self.state)

        phi=np.radians(phi)
        tht = np.radians(tht)
        psi = np.radians(psi)

        position = self.aileron_r.get_position()
        if isinstance(position, np.ndarray):
            delta_a_right = position.item()
        else:
            delta_a_right = position
        
        position = self.aileron_l.get_position()
        if isinstance(position, np.ndarray):
            delta_a_left = position.item()
        else:
            delta_a_left = position

        position = self.elevator.get_position()#-4.4460*np.pi/180
        if isinstance(position, np.ndarray):
            delta_e = position.item()
        else:
            delta_e = position
        
        
        delta_r = (self.rudder.get_position())
        delta_t = self.thruster.get_position()#0.3986
        delta_a = (0.5*(delta_a_left-delta_a_right))

        #print('controls: ',delta_a_left,delta_a_right,delta_e,delta_r,delta_t)
        #print('R: ',R.shape)
        #print('self.Vwi: ',(self.Vwi).shape)
        Vwb = R@self.Vwi


        Vgb=np.array([u,v,w])

        Vab=Vgb-Vwb

        
        Va=np.linalg.norm(Vab)
        self.Va=Va
        self.alpha=np.arctan2(Vab[2],Vab[0])
        self.beta=np.arcsin(Vab[1]/Va)
        
        #print('Vgb:',Vgb)
        #print('Vwb:',Vwb)
        #print('Vab:',Vab)
        #print('Va:',Va)
        #print('alpha:',self.alpha)
        #print('beta: ',self.beta)
        #print('p: ',p)
        #print('q: ',q)
        #print('r: ',r)
        sigma = (1+np.exp(-up.M*(self.alpha-up.alpha0))+np.exp(up.M*(self.alpha+up.alpha0)))/((1+np.exp(-up.M*(self.alpha-up.alpha0)))*(1+np.exp(up.M*(self.alpha+up.alpha0))))
        sigma = np.clip(sigma,0,1)
        #print('sigma:',sigma)

        CLalpha = ((1-sigma)*(up.CL0+up.CL_alpha*self.alpha))+(sigma*2*np.sign(self.alpha)*np.cos((self.alpha))*((np.sin((self.alpha)))**2))
        AR=up.b*up.b/up.S
        CDalpha = up.CDp+(CLalpha**2/(np.pi*up.e*AR))

        # Longitudinal forces and moment
        Flift = (0.5*up.rho_air*Va*Va*up.S)*(CLalpha+(up.CL_q*up.c*q/(2*Va))+(up.CL_delta_e*delta_e))
        Fdrag = (0.5*up.rho_air*Va*Va*up.S)*(CDalpha+(up.CD_q*up.c*q/(2*Va))+(up.CD_delta_e*delta_e))
        mom_m = (0.5*up.rho_air*Va*Va*up.S*up.c)*(up.Cm0+(up.Cm_alpha*self.alpha)+(up.Cm_q*q/(2*Va))+(up.Cm_delta_e*delta_e))
        fx = (Flift*np.sin((self.alpha)))-(Fdrag*np.cos((self.alpha)))
        fz = -(Flift*np.cos((self.alpha)))-(Fdrag*np.sin((self.alpha)))


        #Lateral force and moments
        fy = (0.5*up.rho_air*Va*Va*up.S)*(up.CY0+(up.CY_beta*self.beta)+(up.CY_p*up.b*p/(2*Va))+(up.CY_r*up.b*r/(2*Va))+(up.CY_delta_a*delta_a)+(up.CY_delta_r*delta_r))
        mom_l = (0.5*up.rho_air*Va*Va*up.S*up.b)*(up.Cl0+(up.Cl_beta*self.beta)+(up.Cl_p*up.b*p/(2*Va))+(up.Cl_r*up.b*r/(2*Va))+(up.Cl_delta_a*delta_a)+(up.Cl_delta_r*delta_r))
        mom_n = (0.5*up.rho_air*Va*Va*up.S*up.b)*(up.Cn0+(up.Cn_beta*self.beta)+(up.Cn_p*up.b*p/(2*Va))+(up.Cn_r*up.b*r/(2*Va))+(up.Cn_delta_a*delta_a)+(up.Cn_delta_r*delta_r))

        fa=np.array([[fx],[fy],[fz]])
        #print(fa)
        ma=np.array([[mom_l],[mom_m],[mom_n]])

        g=9.81                                                                                                                          
        f=np.zeros((3, 1))
        fg=np.array([[-mass*g*np.sin((tht))],
                     [mass*g*np.cos((tht))*np.sin((phi))],
                     [mass*g*np.cos((tht))*np.cos((phi))]])
        
        fp = 0.5*up.rho_air*up.Sprop*up.C_prop*np.array([[(up.kmotor*delta_t)**2-Va**2],[0],[0]])
        mp = np.array([[-up.kTp*((up.k_omega*delta_t)**2)],[0],[0]])
        f=fa+fg+fp
        mom = ma+mp

        #print('fa:',fa)
        #print('fg:',fg)
        #print('fp:',fp)
        #print('ma:',ma)
        #print('mp:',mp)
        return f,mom
    
    
    def get_atmos_address(self):
        with open("environment.txt", "r") as file:
            for line in file:
                agent_id, address = line.strip().split(",")
                return address
        return None

    def get_wind_command(self,command):
        message = {"command": command}
        self.windsocket.send_json(message)
        
        response = self.windsocket.recv_json()
        
        if 'wind_state' in response:
            Vw = response['wind_state']
        else:
            print("No wind state in the response.")        

        return Vw
    
    def start(self):
        self.thread.start()
        print("Dynamics running")
 
    
    def stop(self):
        self.running = False
        self.thread.join()
        print("Dynamics Stopped")
