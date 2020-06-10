# -*- coding: utf-8 -*-
"""
Created on Mon Jun 08 14:54:49 2020

@author: stijn
"""

import math as m
import numpy as np
import matplotlib.pyplot as plt

V_l_kts = 3        #[kts] velocity left mlg
V_r_kts = 10        #[kts] velocity right mlg
tmax = 30

#Geometry 
w_mlg = 7.59        #[m] Width between main gears
d_nlg = 16.90       #[m] Distance from MLG to NLG
#Speeds
V_l = V_l_kts*0.5144444             #[m/s] Velocity left mlg
V_r = V_r_kts*0.5144444             #[m/s] Velocity right mlg

#Turn
if V_l<V_r:
    Turn = "left"
    turn = 1
elif V_l==V_r:
    Turn = "no"
else: 
    Turn = "right"
    turn = -1

R_turn      = turn*abs((w_mlg*(V_l+V_r))/(2*(V_l-V_r)))  #Turn radius of the middle point of the MLG
R_turn_r    = R_turn+(w_mlg/2)
R_turn_l    = R_turn-(w_mlg/2)
R_turn_nlg  = m.sqrt(R_turn**2 + d_nlg**2)
angle_nlg   = 90 - np.degrees(m.acos(d_nlg/R_turn_nlg))
V_nlg_kts = (V_r_kts*R_turn_nlg)/R_turn_r

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
while t<tmax:
    if V_r != 0:
        d_trav_mlgr = V_r*dt
        angle_trav_r = angle_trav_r + d_trav_mlgr/R_turn_r
    else:
        plt.plot(rotation_point,'o',color='blue')
    pos_right_x.append(R_turn_r*m.cos(angle_trav_r))
    pos_right_y.append(R_turn_r*m.sin(angle_trav_r))
    
    if V_l != 0:
        d_trav_mlgl = V_l*dt
        angle_trav_l = angle_trav_l + d_trav_mlgl/R_turn_l
    else:
        plt.plot(rotation_point,'o',color='green')
    pos_left_x.append(R_turn_l*m.cos(angle_trav_l))
    pos_left_y.append(R_turn_l*m.sin(angle_trav_l))
    
    t = t+dt

print('Nose vehicle steering angle: ', angle_nlg, 'degrees')
print('Nose vehicle velocity: ', V_nlg_kts, 'knots')

plt.xlim((min([min(pos_left_x),min(pos_right_x)])-2),((max([max(pos_left_x),max(pos_right_x)])+2)))
plt.ylim((min([min(pos_left_y),min(pos_right_y)])-2),((max([max(pos_left_y),max(pos_right_y)])+2)))
plt.gca().set_aspect('equal', adjustable='box')
plt.plot(pos_right_x,pos_right_y,color='blue',label='Right MLG Wheel')
plt.plot(pos_left_x,pos_left_y,color='green', label='Left MLG Wheel')
plt.legend()
plt.xlabel('X-Coordinate')
plt.ylabel('Y-Coordinate')
plt.arrow(pos_left_x[-2],pos_left_y[-2],(pos_left_x[-1]-pos_left_x[-2]),(pos_left_y[-1]-pos_left_y[-2]),width=0.01,color='green')
plt.arrow(pos_right_x[-2],pos_right_y[-2],(pos_right_x[-1]-pos_right_x[-2]),(pos_right_y[-1]-pos_right_y[-2]),width=0.01,color='blue')