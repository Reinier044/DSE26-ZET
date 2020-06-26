# -*- coding: utf-8 -*-
"""
Created on Mon Jun 15 16:58:44 2020

@author: Reinier
"""
import numpy as np
import matplotlib.pyplot as plt
import sys
from Pushback import choose,choose_2,taxi_method


from Performance_Data import Pa_ZET,Pv_ZET,d_ZET,a_eng,max_d_eng,max_v_eng,dt,t_coupling_ZET,t_coupling_CONV
a_ZET = np.array([0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 ,
       0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 ,
       0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 ,
       0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 ,
       0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 ,
       0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 ,
       0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 , 0.1033678 ,
       0.1033678 , 0.1033678 , 0.1033678 , 0.09640421, 0.08890154,
       0.081774  , 0.07499415, 0.06853715, 0.06238047, 0.05650364,
       0.050888  , 0.04551653, 0.04037362, 0.035445  , 0.03071755,
       0.0261792 , 0.02181883, 0.01762616, 0.0135917 , 0.00970667,
       0.00596291])
v_ZET = np.array([0.       , 0.0514444, 0.1028888, 0.1543332, 0.2057776, 0.257222 ,
       0.3086664, 0.3601108, 0.4115552, 0.4629996, 0.514444 , 0.5658884,
       0.6173328, 0.6687772, 0.7202216, 0.771666 , 0.8231104, 0.8745548,
       0.9259992, 0.9774436, 1.028888 , 1.0803324, 1.1317768, 1.1832212,
       1.2346656, 1.28611  , 1.3375544, 1.3889988, 1.4404432, 1.4918876,
       1.543332 , 1.5947764, 1.6462208, 1.6976652, 1.7491096, 1.800554 ,
       1.8519984, 1.9034428, 1.9548872, 2.0063316, 2.057776 , 2.1092204,
       2.1606648, 2.2121092, 2.2635536, 2.314998 , 2.3664424, 2.4178868,
       2.4693312, 2.5207756, 2.57222  , 2.6236644, 2.6751088, 2.7265532,
       2.7779976, 2.829444 ])
# -------------------Input data ZET-system-----------------
if choose == "long":
    #Taxiway from D14 to Polderbaan
    taxiway = np.array([[21.33,110.4, 100.6, 66.5, 1383,120,754,140,280,70,210,40,130,160,2140,130,1690,150,360],
                    [0,2.829444,0,2.829444,0,2.829444,0,2.829444,0,2.829444,0,2.829444,0,2.829444,0,2.829444,0,2.829444,0,]])
    V_fin_segment = v_ZET[-2]           #Speed at the final segment
    
    if choose_2 == 'in':
        taxiway[0] = np.flip(taxiway[0])
        taxiway[1] = np.flip(taxiway[1])
        V_fin_segment = taxiway[1][-2]
    # Taxiway ID from D14 to Polderbaan
    taxiwayid = np.array(['st', 'cr', 'st', 'cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st'])
    
    


# First row is distance of straight part or corner
# If corner, second row gives max velocity in turn-> 10; 15 or 20 [kts]
elif choose == "short":
    # Taxiway from D14 to runway 36C
    taxiway = np.array([[21.33, 110.4, 100.6, 66.5, 1383, 120, 825, 225, 80],
                    [0, 2.829444, 0, 2.829444, 0, 2.829444, 0, 2.829444, 0, 2.829444, 0]])
    V_fin_segment = taxiway[1][-2]-0.115           #Speed at the final segment
    
    if choose_2 == 'in':
        taxiway[0] = np.flip(taxiway[0])
        taxiway[1] = np.flip(taxiway[1])
        V_fin_segment = taxiway[1][-2]
        
    # Taxiway ID from D14 to runway 36C
    taxiwayid = np.array(['st', 'cr', 'st', 'cr', 'st', 'cr', 'st', 'cr', 'st'])
    # Taxiwayid show whether we have straight part (st) or corner (cr)
    
elif choose == "super short":
    taxiway = np.array([[120, 45, 145],
                    [0, 2.829444, 0]])
    V_fin_segment = 2.829444           #Speed at the final segment
    
    taxiwayid = np.array(['st', 'cr', 'st'])

else:
    sys.exit("No valid taxi route.")
    
totaldis = sum(taxiway[0])
    
tarray_pb = np.array([])
sarray_pb = np.array([])
varray_pb = np.array([])
aarray_pb = np.array([])
status_array_pb = np.array([])

PBlength = 115.0

#Pushback code

#initial values
v = 0.01
t = 0
s = 0
a = 0

#performance values
vpb_max = 1.534
dec_pb = 0.75*d_ZET
break_dis = -0.5*(dec_pb)*(vpb_max/dec_pb)**2

#--------------------------Pusback time----------------------------------------
if choose_2 == 'out':
    while s < PBlength:             #Needed distance covered [m]
        for j in range(len(v_ZET)):
            if v<v_ZET[j]:
                a = a_ZET[j]
                break
    
        if v >= vpb_max:                                   # It can never exceed maximum speed
            v = vpb_max
            a = 0                                           # If maximum speed is achieved, a = 0
            
        if s >= PBlength - break_dis:
            a = dec_pb
         
        v = v + a * dt
        s = s + v * dt
        t = t + dt
       
        if v <= 0.07 and a < 0:
            t0 = t
            while t< t0 + 3:
                v = 0
                a = 0
                tarray_pb = np.append(tarray_pb, t)
                sarray_pb = np.append(sarray_pb, s)
                varray_pb = np.append(varray_pb, v)
                aarray_pb = np.append(aarray_pb, a)
                status_array_pb = np.append(status_array_pb,'pb')
                t = t + dt
            break
        # Appending values to arrays
        tarray_pb = np.append(tarray_pb, t)
        sarray_pb = np.append(sarray_pb, s)
        varray_pb = np.append(varray_pb, v)
        aarray_pb = np.append(aarray_pb, a)
        status_array_pb = np.append(status_array_pb,'pb')

#-------------------------Final braking segment--------------------------------
#Brake to runway time arrays conventional taxi
tarray_fin = np.array([])
aarray_fin = np.array([])
varray_fin = np.array([])
sarray_fin = np.array([])
status_array_fin = np.array([])  

t = 0
v = 2.829444 
a = 0
s = 0
while v>0:
    a = d_ZET
    s = s + v*dt
    tarray_fin = np.append(tarray_fin,t)
    aarray_fin = np.append(aarray_fin,a)
    sarray_fin = np.append(sarray_fin,s)
    varray_fin = np.append(varray_fin,v)
    status_array_fin = np.append(status_array_fin,'st')
    t = t + dt
    v = v + a*dt

Fin_braking_dis = s
taxiway[0][-1]=(taxiway[0][-1]-Fin_braking_dis)
#-------------------------Coupling time----------------------------------------
if choose_2 == 'out':
    #coupling time arrays for ZET
    tarray_cp_ZET = tarray_fin
    aarray_cp_ZET = aarray_fin
    varray_cp_ZET = varray_fin
    sarray_cp_ZET = sarray_fin
    status_array_cp_ZET = status_array_fin

elif choose_2 == 'in':
    tarray_cp_ZET = np.array([])
    aarray_cp_ZET = np.array([])
    varray_cp_ZET = np.array([])
    sarray_cp_ZET = np.array([])
    status_array_cp_ZET = np.array([])
    #Fill coupling arrays for ZET


#to verify the right additions to the simulation
#if choose_2 == 'out':
#    print('ZET receives', tarray_pb[-1]+tarray_cp_ZET[-1], 'seconds')
#elif choose_2 == 'in':
#    print('ZET receives', tarray_cp_ZET[-1], 'seconds')
#    


#Coupling time arrays conventional taxi
tarray_cp_CONV = np.array([])
aarray_cp_CONV = np.array([])
varray_cp_CONV = np.array([])
sarray_cp_CONV = np.array([])
parray_cp_CONV = np.array([])
earray_cp_CONV = np.array([])

if choose_2 == 'out':
    #Fill coupling arrays for conventional taxi
    t= 0
    e = 0
    while t < t_coupling_CONV: 
        tarray_cp_CONV = np.append(tarray_cp_CONV,t)
        aarray_cp_CONV = np.append(aarray_cp_CONV,0)
        varray_cp_CONV = np.append(varray_cp_CONV,0)
        sarray_cp_CONV = np.append(sarray_cp_CONV,0)
        t = t + dt
    
    tarray_pb_CONV = np.array([])
    tarray_pb_CONV = np.append(tarray_pb,tarray_cp_CONV+tarray_pb[-1])
    aarray_pb_CONV = np.array([])
    aarray_pb_CONV = np.append(aarray_pb,aarray_cp_CONV)
    varray_pb_CONV = np.array([])
    varray_pb_CONV = np.append(varray_pb,varray_cp_CONV)
    sarray_pb_CONV = np.array([])
    sarray_pb_CONV = np.append(-sarray_pb[::-1],sarray_cp_CONV)
    
print('Taxi length:',sum(taxiway[0])+PBlength)
#
#if choose_2 == 'out': 
#    print('CONV receives', tarray_pb_CONV[-1]+tarray_fin[-1], 'seconds')
#elif choose_2 == 'in':
#    print('CONV receives', 0, 'seconds')

#
#Plots to check taxi
#
#plt.figure()
#plt.subplot(211)
#plt.plot(tarray_pb,varray_pb)
#plt.subplot(212)
#plt.plot(tarray_cp_ZET,sarray_cp_ZET)
#plt.show()