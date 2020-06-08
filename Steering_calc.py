# -*- coding: utf-8 -*-
"""
Created on Mon Jun 08 14:54:49 2020

@author: stijn
"""

import math as m
import numpy as np
import matplotlib.pyplot as plt

#Geometry 
w_mlg = 7.59        #[m] Width between main gears
d_nlg = 16.90       #[m] Distance from MLG to NLG
#Inputs
V_l = 2             #[m/s] Velocity left mlg
V_r = 4             #[m/s] Velocity right mlg

R_turn      = abs((w_mlg*(V_l+V_r))/(2*(V_l-V_r)))  #Turn radius of the middle point of the MLG
R_turn_r    = R_turn+(w_mlg/2)
R_turn_l    = R_turn-(w_mlg/2)
R_turn_nlg  = m.sqrt(R_turn**2 + d_nlg**2)
angle_nlg   = 90 - np.degrees(m.acos(d_nlg/R_turn_nlg))

#PLOTTING
t = 0
dt = 0.1
angle_trav_r = 0
angle_trav_l = 0

rotation_point = [(0,0)]
pos_left_x = [R_turn-(w_mlg/2)]
pos_left_y = [0]
pos_right_x = [R_turn+(w_mlg/2)]
pos_right_y = [0]




#trajectory 
while t<5:
    d_trav_mlgr = V_r*dt
    angle_trav_r = angle_trav_r + d_trav_mlgr/R_turn_r
    pos_right_x.append(R_turn_r*m.cos(angle_trav_r))
    pos_right_y.append(R_turn_r*m.sin(angle_trav_r))
    
    d_trav_mlgl = V_l*dt
    angle_trav_l = angle_trav_l + d_trav_mlgl/R_turn_l
    pos_left_x.append(R_turn_l*m.cos(angle_trav_l))
    pos_left_y.append(R_turn_l*m.sin(angle_trav_l))
    
    t = t+dt

plt.xlim(0,(1.1*R_turn_r))
#plt.gca().set_aspect('equal', adjustable='box')
plt.plot(pos_right_x,pos_right_y)
plt.plot(pos_left_x,pos_left_y)