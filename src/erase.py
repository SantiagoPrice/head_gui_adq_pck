# -*- coding: utf-8 -*-
"""
Created on Tue Dec  5 11:40:20 2023

@author: UTAIL
"""
import numpy as np
# Knee dimentions for knee exoskeleton

# First approach: 2 rectangular bars
curv = .16       #m
L = curv/(np.pi/4) #m
h =1e-3         #m
w =5e-3       #m
offset = 50e-3 #m
E = 3752.16e9      #Pa
S_req = 1.24     #N.m / deg


a=h*w
I_bar=h**3*w/12
I_low=2*I_bar
I_high=2*I_bar+2*(offset/2*a**2)
 
S_low=2*I_low*E/L * (np.pi/180)   #N.m / deg
S_high=2*I_high*E/L * (np.pi/180)   #N.m / deg

print("Deformation :{:.2f}".format(h/2/curv))
print("Compensation ratio:")

print(f"Low: {S_low/S_req}")
print(f"High: {S_high/S_req}")

# Verification of elastic modulus
L=0.325 #m
S_exp=1.24*180/np.pi*L #N/rad*m
E_exp=S_exp*L/(2*I_bar)*10e-9
print("Experimental modulus:{:.2f}".format(E_exp))