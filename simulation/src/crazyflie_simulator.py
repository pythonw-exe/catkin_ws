#!/usr/bin/env python3

import numpy as np
from numpy import sin,cos,tan

class drone_class():
    def __init__(self, m, g, L, I_xx, I_yy, I_zz):
        self.m = m
        self.g = g
        self.I_xx = I_xx
        self.I_yy = I_yy
        self.I_zz = I_zz
        self.L = L

    def rotation_matrix(self, phi, theta, psi):
        self.R = np.array(\
            [cos(theta)*cos(psi), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi)],
            [cos(theta)*sin(psi), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(theta)],
            [-sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)])    

    def position_dynamics(self, phi, theta, psi, f):
        v_x_dot = -f*1/self.m*(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))
        v_y_dot = -f*1/self.m*(sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(theta))
        v_z_dot = self.g - f*(1/self.m)*(cos(phi)*cos(theta))
        return v_x_dot, v_y_dot, v_z_dot

    def orientation_dynamics(self, p, q, r):
        p_dot = (1/self.I_xx) * (q*r*(self.I_yy-self.I_zz))
        q_dot = (1/self.I_yy) * (p*r*(self.I_zz-self.I_xx))
        r_dot = (1/self.I_zz) * (p*q*(self.I_xx-self.I_yy))
        return p_dot, q_dot, r_dot

    def orientation_kinematics(self, p, q, r, phi, theta):
        phi_dot = p+q*tan(theta)*sin(phi)+r*tan(theta)*cos(phi)
        theta_dot = q*cos(phi)-r*sin(phi)
        psi_dot = (q*sin(phi)+r*cos(phi))/cos(theta)
        return phi_dot, theta_dot, psi_dot

    def run_drone():

    