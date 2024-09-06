#Copyright (c) 2024, Ayaz Ahmed
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree. 

import numpy as np
import uav_paramters as up
import zmq
import time
import threading


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.previous_error = 0.0
        self.integral_min = float('-inf')
        self.integral_max = float('inf')
    
    def set_integral_limits(self, integral_min, integral_max):
        self.integral_min = integral_min
        self.integral_max = integral_max

    def compute(self, setpoint, measurement, dt):
        if dt == 0:
            return 0  # Avoid division by zero
        
        error = setpoint - measurement
        self.integral += error * dt
        # Prevent integral windup
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
    
class Controller:
    def __init__(self,dynamics,aileron_l,aileron_r,rudder,elevator,thruster,target):
        self.running =True
        self.lock = threading.Lock()

        self.target = target        #drop location
        self.target_psi = 0.0       #target heading
        self.cruise_height = 100.0  #cruise altitude
        self.drop_height = 10.0     #height at which drop has to made
        self.takeoff_height = 20.0  #where takeoff ends and climb starts
        self.cruise_airspeed = 30.0 #cruising airspeed m/s
        self.delta_tstar = 0.3986   #trim throttle
        self.takeoff_theta = 20 * np.pi/180 # takeoff theta
        self.dwell_speed = 10.0     # airspeed while climbing
        self.climb_speed = 30.0     #airspeed while climbing

        self.internaldynamics = dynamics
        self.aileron_l = aileron_l
        self.aileron_r = aileron_r
        self.rudder = rudder
        self.elevator = elevator
        self.thruster = thruster

        #calculated parameters
        self.a_phi_1=0
        self.a_phi_1=0
        self.a_beta_1=0
        self.a_beta_2=0

        # Tuning parameters for roll
        self.e_phi_max=30.0*np.pi/180
        self.damp_phi = 5.0
        self.damp_xi = 20.0
        self.Ki_phi = 0.005
        self.W_xi = 100.0


        # Tuning parameters for yaw
        self.e_beta_max=50.0*np.pi/180
        self.damp_beta=50

        # Tuning parameters for Velocity hold using throttle
        self.damp_v = 1
        self.omega_n_v = 10.0

        # Tuning parameters for pitch and altitude hold
        self.e_theta_max = 1.0*np.pi/180
        self.damp_theta = 5
        self.damp_h = 1
        self.W_h = 10

        self.omega_n_h=0
        self.omega_n_theta = 0

        # tuning parameter for aispeed hold using pitch
        self.W_V2 = 15.0
        self.damp_V2 = 20.0



        #
        self.Kp_phi = 0.0
        self.Kd_phi = 0.0
        self.Kp_xi = 0.0
        self.Ki_xi = 0.0
        self.Kp_beta = 0.0
        self.Ki_beta = 0.0
        self.Kp_v = 0.0
        self.Ki_v = 0.0
        self.Kp_theta = 0.0
        self.Kd_theta = 0.0
        self.Kp_h = 0.0
        self.Ki_h = 0.0
        self.Kp_V2 = 0.0
        self.Ki_V2 = 0.0

        

        calculate_gains(self,10)
        
        #print('Kp_phi: ',self.Kp_phi)
        #print('Ki_phi: ',self.Ki_phi)
        #print('Kd_phi: ',self.Kd_phi)
        #print('Kp_xi: ',self.Kp_xi)
        #print('Ki_xi: ',self.Ki_xi)
        #print('Kp_beta: ',self.Kp_beta)
        #print('Ki_beta: ',self.Ki_beta)
        print('Kp_v: ',self.Kp_v)
        print('Ki_v: ',self.Ki_v)
        print('Kp_theta: ',self.Kp_theta)
        print('Kd_theta: ',self.Kd_theta)
        print('Kp_h: ',self.Kp_h)
        print('Ki_h: ',self.Ki_h)
        #print('omega_n_theta: ',self.omega_n_theta)

        self.xi_c=0
        self.phi_c=0

        self.pid_beta = PIDController(-self.Kp_beta,-self.Ki_beta,0.0)
        self.pid_phi = PIDController(self.Kp_phi,self.Ki_phi,0.0)
        self.pid_xi = PIDController(self.Kp_xi,self.Ki_xi,0.0)
        self.pid_v = PIDController(self.Kp_v,self.Ki_v,0.0)
        self.pid_h = PIDController(self.Kp_h,self.Ki_h,0.0)
        self.pid_V2 = PIDController(self.Kp_V2,self.Ki_V2,0.0)
        
        self.thread = threading.Thread(target=self.run_controller, args=())
        



    def run_controller(self):
        previous_time = time.time()
        while self.running:
            current_time = time.time()
            dt = current_time - previous_time
            if dt > 0:
                with self.lock:
                    state = self.internaldynamics.state
                    Va=self.internaldynamics.Va
                    beta = self.internaldynamics.beta

                if self.internaldynamics.status == 'CRUISE_UP':
                    delta_r = self.pid_beta.compute(0.0,beta,dt)
                    self.rudder.set_target_position(delta_r)
                    #time.sleep(0.001)
                    self.phi_c = self.pid_xi.compute(self.target_psi,self.internaldynamics.state[8]*np.pi/180,dt)
                    #self.phi_c = 0.01
                    #time.sleep(0.001)
                    delta_a = self.pid_phi.compute(self.phi_c,state[6]*np.pi/180,dt)
                    delta_a-=self.Kd_phi*state[9]
                    self.aileron_l.set_target_position(delta_a)
                    self.aileron_r.set_target_position(-delta_a)
                    #print(self.phi_c,state[6]*np.pi/180)

                    delta_t = self.pid_v.compute(self.cruise_airspeed,Va,dt)
                    self.thruster.set_target_position(delta_t+self.delta_tstar)

                    theta_c = self.pid_h.compute(self.cruise_height,-state[2],dt)
                    #theta_c = 2.3249*np.pi/180.0
                    delta_e = self.Kp_theta*(theta_c-(state[7]*np.pi/180))-(self.Kd_theta*state[10])
                    self.elevator.set_target_position(delta_e)
                    #self.elevator.set_target_position(delta_e-4.4460*np.pi/180)

                    #print(time.time(),theta_c,state[7]*np.pi/180,delta_e)
                
                elif self.internaldynamics.status == 'CRUISE_DOWN':
                    delta_r = self.pid_beta.compute(0.0,beta,dt)
                    self.rudder.set_target_position(delta_r)
                    #time.sleep(0.001)
                    self.phi_c = self.pid_xi.compute(self.target_psi,self.internaldynamics.state[8]*np.pi/180,dt)
                    #self.phi_c = 0.01
                    #time.sleep(0.001)
                    delta_a = self.pid_phi.compute(self.phi_c,state[6]*np.pi/180,dt)
                    delta_a-=self.Kd_phi*state[9]
                    self.aileron_l.set_target_position(delta_a)
                    self.aileron_r.set_target_position(-delta_a)
                    #print(self.phi_c,state[6]*np.pi/180)

                    delta_t = self.pid_v.compute(self.cruise_airspeed,Va,dt)
                    self.thruster.set_target_position(delta_t+self.delta_tstar)

                    theta_c = self.pid_h.compute(self.cruise_height,-state[2],dt)
                    #theta_c = 2.3249*np.pi/180.0
                    delta_e = self.Kp_theta*(theta_c-(state[7]*np.pi/180))-(self.Kd_theta*state[10])
                    self.elevator.set_target_position(delta_e)
                    #self.elevator.set_target_position(delta_e-4.4460*np.pi/180)

                    #print(time.time(),theta_c,state[7]*np.pi/180,delta_e)

                elif self.internaldynamics.status == 'CLIMB':
                    self.target_psi = np.arctan2(self.target[1]-self.internaldynamics.state[1],self.target[0]-self.internaldynamics.state[0])
                    delta_r = self.pid_beta.compute(0.0,beta,dt)
                    self.rudder.set_target_position(delta_r)
                    #time.sleep(0.001)
                    self.phi_c = self.pid_xi.compute(self.target_psi,self.internaldynamics.state[8]*np.pi/180,dt)
                    #self.phi_c = 0.01
                    #time.sleep(0.001)
                    delta_a = self.pid_phi.compute(self.phi_c,state[6]*np.pi/180,dt)
                    delta_a-=self.Kd_phi*state[9]
                    self.aileron_l.set_target_position(delta_a)
                    self.aileron_r.set_target_position(-delta_a)
                    #print(self.phi_c,state[6]*np.pi/180)


                    self.thruster.set_target_position(0.9*up.max_delta_t)

                    theta_c = self.pid_V2.compute(self.climb_speed,Va,dt)
                    #theta_c = 2.3249*np.pi/180.0
                    delta_e = self.Kp_theta*(theta_c-(state[7]*np.pi/180))-(self.Kd_theta*state[10])
                    self.elevator.set_target_position(delta_e)

                    if np.abs(self.internaldynamics.state[2]+self.cruise_height)<10:
                        self.internaldynamics.status = 'CRUISE_UP'
                        print('Starting Cruise')
                    #self.elevator.set_target_position(delta_e-4.4460*np.pi/180)

                    #print(time.time(),theta_c,state[7]*np.pi/180,delta_e)

                elif self.internaldynamics.status == 'TAKEOFF':
                    delta_r = self.pid_beta.compute(0.0,beta,dt)
                    self.rudder.set_target_position(delta_r)
                    #time.sleep(0.001)
                    self.phi_c = self.pid_xi.compute(self.target_psi,self.internaldynamics.state[8]*np.pi/180,dt)
                    #self.phi_c = 0.01
                    #time.sleep(0.001)
                    delta_a = self.pid_phi.compute(self.phi_c,state[6]*np.pi/180,dt)
                    delta_a-=self.Kd_phi*state[9]
                    self.aileron_l.set_target_position(delta_a)
                    self.aileron_r.set_target_position(-delta_a)
                    #print(self.phi_c,state[6]*np.pi/180)


                    self.thruster.set_target_position(0.9*up.max_delta_t)
                    
                    #delta_t = self.pid_v.compute(self.dwell_speed,Va,dt)
                    #self.thruster.set_target_position(delta_t+self.delta_tstar)

                    #theta_c = self.pid_V2.compute(self.climb_speed,Va,dt)
                    gamma=10*np.pi/180

                    theta_c = self.internaldynamics.alpha+gamma
                    #print(theta_c)
                    delta_e = self.Kp_theta*(theta_c-(state[7]*np.pi/180))-(self.Kd_theta*state[10])
                    self.elevator.set_target_position(delta_e)
                    #self.elevator.set_target_position(delta_e-4.4460*np.pi/180)

                    #print(time.time(),theta_c,state[7]*np.pi/180,delta_e)

                    if -self.internaldynamics.state[2]>10:
                        self.internaldynamics.status = 'HEADING'
                        self.heading_height = -self.internaldynamics.state[2]+10
                        self.heading_velocity = self.internaldynamics.Va+5
                        calculate_gains(self,self.heading_velocity)
                        print('Starting heading correction')
                
                elif self.internaldynamics.status == 'HEADING':
                    self.target_psi = np.arctan2(self.target[1]-self.internaldynamics.state[1],self.target[0]-self.internaldynamics.state[0])
                    #print(self.target_psi*180/np.pi)
                    delta_r = self.pid_beta.compute(0.0,beta,dt)
                    self.rudder.set_target_position(delta_r)
                    #time.sleep(0.001)
                    self.phi_c = self.pid_xi.compute(self.target_psi,self.internaldynamics.state[8]*np.pi/180,dt)
                    #self.phi_c = 0.01
                    #time.sleep(0.001)
                    delta_a = self.pid_phi.compute(self.phi_c,state[6]*np.pi/180,dt)
                    delta_a-=self.Kd_phi*state[9]
                    self.aileron_l.set_target_position(delta_a)
                    self.aileron_r.set_target_position(-delta_a)
                    #print(self.phi_c,state[6]*np.pi/180)

                    delta_t = self.pid_v.compute(self.heading_velocity,Va,dt)
                    self.thruster.set_target_position(delta_t+self.delta_tstar)

                    theta_c = self.pid_h.compute(self.heading_height,-state[2],dt)
                    #theta_c = 2.3249*np.pi/180.0
                    delta_e = self.Kp_theta*(theta_c-(state[7]*np.pi/180))-(self.Kd_theta*state[10])
                    self.elevator.set_target_position(delta_e)
                    #self.elevator.set_target_position(delta_e-4.4460*np.pi/180)

                    #print(time.time(),theta_c,state[7]*np.pi/180,delta_e)
                    if np.abs(self.target_psi-self.internaldynamics.state[8]*np.pi/180)<10*np.pi/180:
                        self.internaldynamics.status = 'CLIMB'
                        self.climb_speed=self.internaldynamics.Va
                        print('Staring Climb')
                            
                    
            previous_time = current_time
            time.sleep(0.01)  # Update at 100 Hz

    def start(self):
        self.thread.start()
        self.internaldynamics.controller_running=1.0
        print("Controller running")
 
    
    def stop(self):
        self.running = False
        self.thread.join()
        print("Controller Stopped")
        

def calculate_gains(controller,desVa):
    CP_p = (up.C3*up.Cl_p)+(up.C4*up.Cn_p)
    CP_delta_a = (up.C3*up.Cl_delta_a)+(up.C4*up.Cn_delta_a)
    a_phi_1 = (-0.25*up.rho_air*up.S*up.b*up.b*CP_p*desVa)
    a_phi_2 = (0.5*up.rho_air*up.S*up.b*CP_delta_a*(desVa**2))

    omega_n_phi = np.sqrt(np.abs(a_phi_1)*up.max_delta_a/controller.e_phi_max)
    omega_n_xi = omega_n_phi/controller.W_xi

    controller.Kp_phi = up.max_delta_a*np.sign(a_phi_2)/controller.e_phi_max
    controller.Kd_phi = ((2*controller.damp_phi*omega_n_phi)-a_phi_1)/a_phi_2
    
    Vg = np.linalg.norm(controller.internaldynamics.state[3:6])
    controller.Kp_xi = 2*controller.damp_xi*omega_n_xi*Vg/up.g
    controller.Ki_xi = (omega_n_xi**2)*Vg/up.g

    a_beta_1 = -0.5*up.rho_air*desVa*up.S*up.CY_beta/controller.internaldynamics.total_mass
    a_beta_2 = 0.5*up.rho_air*desVa*up.S*up.CY_delta_r/controller.internaldynamics.total_mass

    controller.Kp_beta = up.max_delta_r*np.sign(a_beta_2)/controller.e_beta_max
    controller.Ki_beta = (1/a_beta_2)*((((a_beta_1)+(a_beta_2*controller.Kp_beta))/(2*controller.damp_beta))**2)
    
    # for velocity hold using throttle
    alpha_star = 2.3249*np.pi/180
    delta_t_star = 0.3986
    chi_star = 0
    theta_star = 2.3249*np.pi/180
    delta_e_star = -4.4460*np.pi/180 

    aV1 = (up.rho_air * desVa * up.S / controller.internaldynamics.total_mass) * (up.CD0 + up.CD_alpha * alpha_star + up.CD_delta_e * delta_e_star) + (up.rho_air * up.Sprop / up.m) * up.C_prop * desVa
    aV2 = (up.rho_air * up.Sprop / controller.internaldynamics.total_mass) * up.C_prop * (up.kmotor**2) * delta_t_star
    aV3 = up.g * np.cos(theta_star - chi_star)

    controller.Kp_v = ((2*controller.damp_v*controller.omega_n_v)-aV1)/aV2
    controller.Ki_v = (controller.omega_n_v**2)/aV2

    # for attitude hold
    a_theta1 = -(up.rho_air * desVa**2 * up.c * up.S) / (2 * up.J_y) * up.Cm_q * (up.c / (2 * desVa))
    a_theta2 = -(up.rho_air * desVa**2 * up.c * up.S) / (2 * up.J_y) * up.Cm_alpha
    a_theta3 = (up.rho_air * desVa**2 * up.c * up.S) / (2 * up.J_y) * up.Cm_delta_e 

    controller.Kp_theta = up.max_delta_e*np.sign(a_theta3)/controller.e_theta_max
    controller.omega_n_theta = np.sqrt(a_theta2+np.abs(controller.Kp_theta))
    controller.Kd_theta = ((2*controller.damp_theta*controller.omega_n_theta)-a_theta1)/a_theta3

    controller.K_theta_DC = controller.Kp_theta*a_theta3/(a_theta2+(controller.Kp_theta*a_theta3))

    #for altitude hold using commanded pitch    
    controller.omega_n_h = controller.omega_n_theta/controller.W_h
    controller.Ki_h = (controller.omega_n_h**2)/(controller.K_theta_DC*desVa)
    controller.Kp_h = 2*controller.damp_h*controller.omega_n_h/(controller.K_theta_DC*desVa)

    # for velocity hold using pitch
    omega_n_V2 = controller.omega_n_theta/controller.W_V2
    controller.Kp_V2 = (aV1-(2*controller.damp_V2*omega_n_V2))/(controller.K_theta_DC*up.g)
    controller.Ki_V2 = -(omega_n_V2**2)/(controller.K_theta_DC*up.g)
