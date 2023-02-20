import math
import numpy as np

class kinematic():
    def __init__(self):
        self.r = 0.0625 # [m]
        self.d = 0.55 # [m]

    def forward_matrix(self):
        J_for = (0.7071/self.r)*np.array([
        [1, 1, -1, -1],
        [1, -1, -1, 1],
        [-1/(2*self.d), -1/(2*self.d), -1/(2*self.d), -1/(2*self.d)]
        ])
        return J_for
    
    def inverse_matrix(self):
        J_inv = (self.r/(4*0.7071))*np.array([
        [1,1,-2*self.d],
        [1,-1,-2*self.d],
        [-1,-1,-2*self.d],
        [-1,1,-2*self.d]
        ])
        return J_inv
    
    def inverse_kinematic(self, vx, vy, vth):
        w = self.inverse_matrix() @ np.array([vx, vy, vth])
        return w[0], w[1], w[2], w[3]
    
    def forward_kinematic(self, w1, w2, w3, w4):
        vec = self.forward_matrix() @ np.array([w1, w2, w3, w4])
        return vec[0], vec[1], vec[2]
    
    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value