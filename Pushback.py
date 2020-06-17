# -*- coding: utf-8 -*-
"""
Created on Mon Jun 15 16:58:44 2020

@author: Reini
"""
import numpy as np
from Performance_Data import a_ZET,v_ZET,Pa_ZET,Pv_ZET,d_ZET,a_eng,max_d_eng,max_v_eng,dt,t_coupling_ZET,t_coupling_CONV
import matplotlib.pyplot as plt

choose = "long" 
choose_2 = 'in'


# -------------------Input data ZET-system-----------------
if choose == "long":
    #Taxiway from D14 to Polderbaan
    taxiway = np.array([[21.33,110.4, 100.6, 66.5, 1383,120,754,140,280,70,210,40,130,160,2140,130,1690,150,360],
                    [0,5.1444,0,5.1444,0,5.1444,0,10.2889,0,5.1444,0,10.2889,0,7.7167,0,5.1444,0,5.1444,0,]])
    V_fin_segment = v_ZET[-1]           #Speed at the final segment
    
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
                    [0, 5.1444, 0, 5.1444, 0, 5.1444, 0, 10.2889, 0, 10.2889, 0]])
    V_fin_segment = v_ZET[-1]           #Speed at the final segment
    
    if choose_2 == 'in':
        taxiway[0] = np.flip(taxiway[0])
        taxiway[1] = np.flip(taxiway[1])
        V_fin_segment = taxiway[1][-2]
        
    # Taxiway ID from D14 to runway 36C
    taxiwayid = np.array(['st', 'cr', 'st', 'cr', 'st', 'cr', 'st', 'cr', 'st'])
    # Taxiwayid show whether we have straight part (st) or corner (cr)

else:
    sys.exit("No valid taxi route.")
    
totaldis = sum(taxiway[0])
    
tarray_pb = np.array([])
sarray_pb = np.array([])
varray_pb = np.array([])
aarray_pb = np.array([])

PBlength = 11.5
power_cp = 50 #power needed for coupling
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
                a = 0.5*a_ZET[j]
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
                t = t + dt
            break
        # Appending values to arrays
        tarray_pb = np.append(tarray_pb, t)
        sarray_pb = np.append(sarray_pb, s)
        varray_pb = np.append(varray_pb, v)
        aarray_pb = np.append(aarray_pb, a)

#-------------------------Final braking segment--------------------------------
#Brake to runway time arrays conventional taxi
tarray_fin = np.array([])
aarray_fin = np.array([])
varray_fin = np.array([])
sarray_fin = np.array([])
parray_fin = np.array([])
earray_fin = np.array([])  

t = 0
p = 0
e = 0
v = V_fin_segment
a = 0
s = 0
while v>0:
    a = d_ZET
    s = s + v*dt
    tarray_fin = np.append(tarray_fin,t)
    aarray_fin = np.append(aarray_fin,a)
    sarray_fin = np.append(sarray_fin,s)
    varray_fin = np.append(varray_fin,v)
    parray_fin = np.append(parray_fin,p)
    earray_fin = np.append(earray_fin,e)
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
    parray_cp_ZET = parray_fin
    earray_cp_ZET = earray_fin
    
    #Fill coupling arrays for ZET
    t = tarray_fin[-1]
    e = earray_fin[-1]
    v = varray_fin[-1]
    a = 0
    s = sarray_fin[-1]
    
    while t <= tarray_fin[-1]+t_coupling_ZET: 
        v = 0
        s = s + v*dt
        tarray_cp_ZET = np.append(tarray_cp_ZET,t)
        aarray_cp_ZET = np.append(aarray_cp_ZET,a)
        varray_cp_ZET = np.append(varray_cp_ZET,v)
        sarray_cp_ZET = np.append(sarray_cp_ZET,s)
        parray_cp_ZET = np.append(parray_cp_ZET,power_cp)    
        earray_cp_ZET = np.append(earray_cp_ZET,e)
        e = e+ power_cp*dt
        t = t + dt

elif choose_2 == 'in':
    tarray_cp_ZET = np.array([])
    aarray_cp_ZET = np.array([])
    varray_cp_ZET = np.array([])
    sarray_cp_ZET = np.array([])
    parray_cp_ZET = np.array([])
    earray_cp_ZET = np.array([])
    
    #Fill coupling arrays for ZET
    t = 0
    e = 0
    v = 0
    a = 0
    s = 0
    
    while t <= t_coupling_ZET: 
        v = 0
        s = s + v*dt
        tarray_cp_ZET = np.append(tarray_cp_ZET,t)
        aarray_cp_ZET = np.append(aarray_cp_ZET,a)
        varray_cp_ZET = np.append(varray_cp_ZET,v)
        sarray_cp_ZET = np.append(sarray_cp_ZET,s)
        parray_cp_ZET = np.append(parray_cp_ZET,power_cp)    
        earray_cp_ZET = np.append(earray_cp_ZET,e)
        e = e+ power_cp*dt
        t = t + dt



if choose_2 == 'out':
    print('ZET receives', tarray_pb[-1]+tarray_cp_ZET[-1], 'seconds')
elif choose_2 == 'in':
    print('ZET receives', tarray_cp_ZET[-1], 'seconds')
    


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
        parray_cp_CONV = np.append(parray_cp_CONV,power_cp)    
        earray_cp_CONV = np.append(earray_cp_CONV,e)
        e = e + power_cp*dt
        t = t + dt
    
    tarray_pb_CONV = np.array([])
    tarray_pb_CONV = np.append(tarray_pb,tarray_cp_CONV+tarray_pb[-1])
    aarray_pb_CONV = np.array([])
    aarray_pb_CONV = np.append(aarray_pb,aarray_cp_CONV)
    varray_pb_CONV = np.array([])
    varray_pb_CONV = np.append(varray_pb,varray_cp_CONV)
    sarray_pb_CONV = np.array([])
    sarray_pb_CONV = np.append(-sarray_pb[::-1],sarray_cp_CONV)

if choose_2 == 'out': 
    print('CONV receives', tarray_pb_CONV[-1]+tarray_fin[-1], 'seconds')
elif choose_2 == 'in':
    print('CONV receives', 0, 'seconds')

#
#Plots to check taxi
#
#plt.figure()
#plt.subplot(211)
#plt.plot(tarray_pb,varray_pb)
#plt.subplot(212)
#plt.plot(tarray_cp_ZET,sarray_cp_ZET)
#plt.show()