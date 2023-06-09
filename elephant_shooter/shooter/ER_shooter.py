#!/usr/bin/env python3
import numpy as np
import getpass
import yaml
username = getpass.getuser()

def read_one_block_of_yaml_data(filename):
    with open(f'{filename}','r') as f:
        output = yaml.safe_load(f) 
    return output['distance']

class shooter():
    def __init__(self, distance):
        self.distance = distance
        self.param = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0,0.0, 0.0, 0.0, 0.0, 0.0]
        self.file = f'/home/{username}/elephant_ws/src/elephant_shooter/config/param.yaml'
        self.param = read_one_block_of_yaml_data(self.file)
        print(self.param)
        # Parameters
        self.d_pulley_1 = 0.02
        self.d_pulley_2 = 0.04
        self.d_roller = 0.051

    def velocity_to_can(self, velocity):
        rps2 = velocity*2/(self.d_roller)
        rps1 = rps2*(self.d_pulley_2/self.d_pulley_1)
        v_to_roller = rps1
        if (v_to_roller > 1500):
            v_in = 1500
        v_o = self.map(v_to_roller, 0, 1500, 0, 65535)
        return v_o
    
    def velocity(self, distance):
        theta = np.pi/4
        h = 0.0
        if(distance > 0.6 and distance<=1.7):
            h = -1 + 0.58
        elif(distance >=2.5 and distance<3.8):
            h = -1.2 + 0.58
            distance = distance * 4
        elif(distance >=3.8 and distance <=4.1):
            h = -1.9 + 0.58
            distance = distance * 4.5
        v = np.sqrt(9.8*self.distance**2/(self.distance*np.tan(theta) - h))
        return v

    def map(self, Input, Min_Input, Max_Input, Min_Output, Max_Output):
        value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
        return value
    
    def shooter(self):
        X_in = self.distance
        if (X_in>=0.9 and X_in<1):
            X_in = X_in*self.param[0]
            #X_in = X_in*3.1
        elif(X_in>=1 and X_in <=1.05):
            X_in = X_in*self.param[1]
            #X_in = X_in*3.25
        elif(X_in>1.05 and X_in <=1.1):
            X_in = X_in*self.param[2]
            #X_in = X_in*3.05
        elif (X_in>1.1 and X_in<=1.2):
            X_in = X_in*self.param[3]
            #X_in = X_in*2
        elif(X_in>1.2 and X_in<=1.3):
<<<<<<< HEAD
            X_in = X_in*self.param[4]
=======
            a = self.map(self.adjust, -1, 1, 1.95,2.1)
            X_in = X_in*a
>>>>>>> 91752ec763aa4ee424cc9ad63fca54441353c84c
            #X_in = X_in*1.9
        elif(X_in>1.3 and X_in<=1.4):
            X_in = X_in*self.param[5]
            #X_in = X_in * 2.1
        elif(X_in> 1.4 and X_in<=1.5):
            a = self.map(X_in, 1.5, 0.5, self.param[6], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.5, 1.5)
        elif(X_in>1.5 and X_in <= 1.7):
            a = self.map(X_in, 1.5, 0.5, self.param[7], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.95, 1.5)
        elif(X_in>1.7 and X_in<=1.8):
            a = self.map(X_in, 1.5, 0.5, self.param[8], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.94, 1.5)
        elif(X_in>1.8 and X_in <=2):
            a = self.map(X_in, 1.5, 0.5, self.param[9], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.12, 1.5)
        elif(X_in>2 and X_in<2.2):
            a = self.map(X_in, 1.5, 0.5, self.param[10], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5)
        elif(X_in>=2.2 and X_in<=2.3):
            a = self.map(X_in, 1.5, 0.5, self.param[11], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.09, 1.5)
        elif(X_in>2.3 and X_in<=2.5):
            a = self.map(X_in, 1.5, 0.5, self.param[12], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.7, 1.5)
        elif(X_in>2.5 and X_in<=2.6):
            a = self.map(X_in, 1.5, 0.5, self.param[13], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 3.73, 1.5)    
        elif(X_in>2.6 and X_in<=2.7):
<<<<<<< HEAD
            a = self.map(X_in, 1.5, 0.5, self.param[14], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.8, 1.5)
        elif(X_in>2.7 and X_in<=2.8):
            a = self.map(X_in, 1.5, 0.5, self.param[15], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.7, 1.5) 
        elif(X_in>2.8 and X_in<=2.9):
            a = self.map(X_in, 1.5, 0.5, self.param[16], 1.5)
=======
            a = self.map(self.adjust, -1, 1, self.map(X_in, 1.5, 0.5, 3.3, 1.5),self.map(X_in, 1.5, 0.5, 3.4, 1.5))
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.8, 1.5)
        elif(X_in>2.7 and X_in<=2.8):
            a = self.map(self.adjust, -1, 1, self.map(X_in, 1.5, 0.5, 3.8, 1.5),self.map(X_in, 1.5, 3.9, 3, 1.5))
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.7, 1.5) 
        elif(X_in>2.8 and X_in<=2.9):
            a = self.map(self.adjust, -1, 1, self.map(X_in, 1.5, 0.5, 3.9, 1.5),self.map(X_in, 1.5, 4, 3, 1.5))
>>>>>>> 91752ec763aa4ee424cc9ad63fca54441353c84c
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5) 
        elif(X_in>3.2 and X_in<=3.5):
            a = self.map(X_in, 1.5, 0.5, self.param[17], 1.5)
            X_in = a
            #X_in = self.map(X_in, 1.5, 0.5, 2.65, 1.5)
        elif(X_in>3.5 and X_in<=3.8):
            a = self.map(X_in, 1.5, 0.5, self.param[18], 1.5)
            X_in = a
            #X_in = self.map(X_in, 2.25, 0.5, 3.9, 1.5)
        elif(X_in>3.8 and X_in<=4.1):
<<<<<<< HEAD
            a = self.map(X_in, 1.5, 0.5, self.param[19], 1.5)
            X_in = a
            #X_in = self.map(X_in, 2.35, 0.5, 4.05, 1.5)
        elif(X_in>4.1 and X_in<=4.4):
            a = self.map(X_in, 1.5, 0.5, self.param[20], 1.5)
=======
            a = self.map(self.adjust, -1, 1, self.map(X_in, 1.5, 0.5, 4, 1.5),self.map(X_in, 1.5, 0.5, 4.1, 1.5))
            X_in = a
            #X_in = self.map(X_in, 2.35, 0.5, 4.05, 1.5)
        elif(X_in>4.1 and X_in<=4.4):
            a = self.map(self.adjust, -1, 1, self.map(X_in, 1.5, 0.5, 3.8, 1.5),self.map(X_in, 1.5, 0.5, 3.9, 1.5))
>>>>>>> 91752ec763aa4ee424cc9ad63fca54441353c84c
            X_in = a
            #X_in = self.map(X_in, 2.46, 0.5, 4.15, 1.5)
        elif(X_in>=5.5 and X_in<5.6):
            a = self.map(X_in, 1.5, 0.5, self.param[21], 1.5)
            X_in = a
            #X_in = self.map(X_in,1.5, 0.5,3.5, 1.5)
        elif(X_in>0.7 and X_in<0.9):
<<<<<<< HEAD
            X_in = X_in*self.param[22]
=======
            a = self.map(self.adjust, -1,1,3.5,3.6)
            X_in = X_in*a
>>>>>>> 91752ec763aa4ee424cc9ad63fca54441353c84c
            #X_in = 3.4*X_in 
        elif(X_in>0.6 and X_in<=0.7):
            X_in = X_in*self.param[23]
            #X_in = 3.9*X_in
        else:
            X_in = X_in*self.param[24]
            #X_in = 3.5*X_in
        self.distance = X_in
        v = self.velocity(X_in)
        
        rps = self.velocity_to_can(v)
        return rps


    
