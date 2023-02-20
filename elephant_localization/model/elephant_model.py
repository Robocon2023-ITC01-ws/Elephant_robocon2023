import math
import numpy as np

class ElephantModel():
    def __init__(self):
        self.r = 0.0625 #[m]
        self.d = 0.55 # [m]
        
    def forward_matrix(self):
        J_for = (0.7071/self.r)*np.array([
        [1, 1, -1, -1],
        [1, -1, -1, 1],
        [1/(2*self.d), 1/(2*self.d), 1/(2*self.d), 1/(2*self.d)]
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
    
    def rotation_matrix(self, angle):
        rot = np.array([
            [math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle), math.cos(angle), 0],
            [0,     0,      1]
        ])
        return rot
    
    def forward_kinematic(self, w1, w2, w3, w4, angle):
        for_vec = self.rotation_matrix(angle).T @ self.forward_matrix() @ np.array([w1, w2, w3, w4])
        return for_vec
    
    def inverse_kinematic(self, vx, vy, vth):
        inv_vec = self.inverse_matrix() @ np.array([vx, vy, vth])
        return inv_vec
    
    def map(self,Input, min_input, max_input, min_output, max_output):
        value = ((Input - min_input)*(max_output-min_output)/(max_input - min_input) + min_output)
        return value

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return qx, qy, qz, qw

    def euler_from_quaternion(self, x, y, z, w):
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)
		
        t2 = 2.0 * (w * y - z * x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)
		
        t3 = 2.0 * (w * z +x * y)
        t4 = 1.0 - 2.0*(y * y + z * z)
        yaw = math.atan2(t3, t4)
		
		#if yaw < 0:
		#	yaw = self.map(yaw, -3.1399, -0.001, 3.1399, 6.2799)
		
        return roll, pitch, yaw


    