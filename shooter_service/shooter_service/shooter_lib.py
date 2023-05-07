#!/usr/bin/env python3
import numpy as np

class shooter():
    def __init__(self,type, type_pole, distance):
        self.distance = distance
        self.type_pole = type_pole
        self.type = type

    def map(self, Input, Min_Input, Max_Input, Min_Output, Max_Output):
        value =  ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output)
        return value
    
    def robot_type(self):
        d_pulley_1 = 0.0
        d_pulley_2 = 0.0
        d_roller = 0.0
        if self.type == "EE":
            d_pulley_1 = 0.02
            d_pulley_2 = 0.04
            d_roller = 0.051

        elif self.type == "ER":
            d_roller = 0.04
            d_pulley_1 = 0.016
            d_pulley_2 = 0.04

        return d_roller, d_pulley_1, d_pulley_2
    
    def velocity(self):
        theta = np.pi/4
        h = 0.0
        distance = 0.0
        if self.type == "ER":
            if self.type_pole == "type_1":
                h =-1.3
            elif self.type_pole =="type_2":
                h = -0.6
        elif self.type == "EE":
            if self.type_pole =="type_1":
                h = -1.9 + 0.58
                distance = distance*4.5
            elif self.type_pole =="type_2":
                h = -1+0.58
            elif self.type_pole =="type_3":
                h = -1.2+0.58
                distance = distance*4

        v = np.sqrt(9.8*self.distance**2/(self.distance*np.tan(theta) - h))
        return v
    
    def shooter1(self):
        dr, d1, d2, = self.robot_type()
        v = self.velocity()
        X_in = self.distance
        if self.type == "ER":
            if (X_in>=0.9 and X_in<1):
                X_in = X_in*3.55
            elif(X_in>=1 and X_in <1.05):
                X_in = X_in*3.35
            elif(X_in>1.05 and X_in <=1.1):
                X_in = X_in*3
            elif (X_in>1.1 and X_in<=1.2):
                X_in = X_in*2.805
            elif(X_in>1.2 and X_in<=1.3):
                X_in = X_in*2.605
            elif(X_in>1.3 and X_in<=1.4):
                X_in = X_in * 2.402
            elif(X_in> 1.4 and X_in<=1.5):
                X_in = self.map(X_in, 1.5, 0.5, 3.37, 1.5)
            elif(X_in>1.5 and X_in <= 1.7):
                X_in = self.map(X_in, 1.5, 0.5, 3.33, 1.5)
            elif(X_in>1.7 and X_in<=1.8):
                X_in = self.map(X_in, 1.5, 0.5, 3.2, 1.5)
            elif(X_in>1.8 and X_in <=2):
                X_in = self.map(X_in, 1.5, 0.5, 3.12, 1.5)
            elif(X_in>2 and X_in<2.2):
                X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5)
            elif(X_in>=2.2 and X_in<=2.3):
                X_in = self.map(X_in, 1.5, 0.5, 3.11, 1.5)
            elif(X_in>2.3 and X_in<=2.5):
                X_in = self.map(X_in, 1.5, 0.5, 3.12, 1.5)
            elif(X_in>2.5 and X_in<=2.6):
                X_in = self.map(X_in, 1.5, 0.5, 3.106, 1.5)    
            elif(X_in>2.6 and X_in<=2.8):
                X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5) 
            elif(X_in>3.2 and X_in<=3.5):
                X_in = self.map(X_in, 2.15, 0.5, 3.9, 1.5)
            elif(X_in>3.5 and X_in<=3.8):
                X_in = self.map(X_in, 2.25, 0.5, 3.9, 1.5)
            elif(X_in>3.8 and X_in<=4.1):
                X_in = self.map(X_in, 2.35, 0.5, 4.05, 1.5)
            elif(X_in>4.1 and X_in<=4.4):
                X_in = self.map(X_in, 2.46, 0.5, 4.15, 1.5)
            else:
                X_in = 3.3*X_in
        elif (self.type == "EE"):
            if (X_in>=0.9 and X_in<1):
                X_in = X_in*2.65
            elif(X_in>=1 and X_in <1.05):
                X_in = X_in*2.56
            elif(X_in>1.05 and X_in <=1.1):
                X_in = X_in*2.3879
            elif (X_in>1.1 and X_in<=1.2):
                X_in = X_in*2.2
            elif(X_in>1.2 and X_in<=1.3):
                X_in = X_in*2.1
            elif(X_in>1.3 and X_in<=1.4):
                X_in = X_in * 2.08
            elif(X_in> 1.4 and X_in<=1.5):
                X_in = self.map(X_in, 1.5, 0.5, 3, 1.5)
            elif(X_in>1.5 and X_in <= 1.7):
                X_in = self.map(X_in, 1.5, 0.5, 2.95, 1.5)
            elif(X_in>1.7 and X_in<=1.8):
                X_in = self.map(X_in, 1.5, 0.5, 2.94, 1.5)
            elif(X_in>1.8 and X_in <=2):
                X_in = self.map(X_in, 1.5, 0.5, 3.12, 1.5)
            elif(X_in>2 and X_in<2.2):
                X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5)
            elif(X_in>=2.2 and X_in<=2.3):
                X_in = self.map(X_in, 1.5, 0.5, 3.11, 1.5)
            elif(X_in>2.3 and X_in<=2.5):
                X_in = self.map(X_in, 1.5, 0.5, 3.703, 1.5)
            elif(X_in>2.5 and X_in<=2.6):
                X_in = self.map(X_in, 1.5, 0.5, 3.2, 1.5)    
            elif(X_in>2.6 and X_in<=2.8):
                X_in = self.map(X_in, 1.5, 0.5, 3.1, 1.5) 
            elif(X_in>3.2 and X_in<=3.5):
                X_in = self.map(X_in, 1.5, 0.5, 4.7, 1.5)
            elif(X_in>3.5 and X_in<=3.8):
                X_in = self.map(X_in, 2.25, 0.5, 3.9, 1.5)
            elif(X_in>3.8 and X_in<=4.1):
                X_in = self.map(X_in, 2.35, 0.5, 4.05, 1.5)
            elif(X_in>4.1 and X_in<=4.4):
                X_in = self.map(X_in, 2.46, 0.5, 4.15, 1.5)
            elif(X_in>=5.5 and X_in<5.6):
                X_in = self.map(X_in,1.5, 0.5,3.5, 1.5)
            else:
                X_in = 4*X_in
        self.distance = X_in
        v = self.velocity()
        rps2 = v*2/(dr)
        rps1 = rps2*(d2/d1)
        V_to_roller = rps1
        if(V_to_roller >1500):
            V_to_roller = 1500
        # V_o = (int)(self.map(V_to_roller, 0, 1500, 0, 65535))
        return V_to_roller


    
    
    
    
        

   
