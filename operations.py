# -*- coding: utf-8 -*-
"""
Created on Wed May 13 08:40:14 2020

@author: Julian
"""

import numpy as np
import matplotlib.pyplot as plt

#-------------------Input data-----------------

max_a = 1.3     #Maximum acceleration
max_d = -1.5       #Maximum deceleration -> should be negative valu5.144e!
max_v = 12.86     #Maximum achievable velocity -> 30 kts (15.433)
#max_v = 10.29

v_cr = 5.144        #Limit on speed on turns (approx 10 kts)

#--------------------Taxiway-------------------

taxiway = np.array([[21.33,35.52,31.68,43.17,105.66,60.91,1383,120,950,80,60],
                    [0,38.8,0,44.8,0,49.5,0,105,0,43.6,52.6]])
#First row is distance of straight part or corner
#If corner, second row gives turn radius -> otherwise 0

taxiwayid = np.array(['st','cr','st','cr','st','cr','st','cr','st','cr','cr'])
#Taxiwayid show whether we have straight part (st) or corner (cr)

#--------------------Code----------------------

#Simulation parameters
t = 0               #Starting time
dt = 0.01          #Time step

#Initial conditions
v = 0               #Starting velocity
s = 0               #Starting distance
ind = 0             #Index value

#Storing arrays
tarray = np.array([0])
sarray = np.array([0])
varray = np.array([0])
aarray = np.array([max_a])

#Simulation
#Only works when you start with straight distance, otherwise we need small modification 

for i in range(len(taxiwayid)):
    
    if taxiwayid[i]=='st':                                  #If we have straight part   
        
        #print('The ',i,'th part is a straight part')
        indstart = ind                                      #Starting index in while loop
        v = varray[indstart]                                #Starting velocity in straight part
            
        while s<(taxiway[0][i]+sarray[indstart]):
                 
            a = max_a                                       #Speeding up with max acceleration    
            v = v + a*dt 
            if v > max_v:
                v = max_v
            s = s + v*dt
            t = t + dt
            ind = ind + 1
            
            #Appending values to arrays
            tarray = np.append(tarray,t)
            sarray = np.append(sarray,s)
            varray = np.append(varray,v)
            aarray = np.append(aarray,a)
            
        if v>v_cr:                                          #If velocity at end of straight part is too high
            
            #Braking
            t_braking = (v_cr - v)/max_d
            #print('Is the time for braking,',t_braking,', reasonable?') 
            
            indnew = ind - int(round(t_braking/dt))         #Go back in time
            a = max_d                                       #Use maximum deceleration for braking
            
            v = varray[indnew]                          #Starting value in this while loop
            s = sarray[indnew]                          #Starting value in this while loop
            t = tarray[indnew]                          #Starting value in this while loop
            
            while sarray[indnew]<=(taxiway[0][i]+sarray[indstart]):     #Of course we still need to cover all distances
               
                v = v + a*dt 
                if v < v_cr:                                #For now, as first order estimate, if we brake sufficiently a small part has a velocity of v_cr before turn
                    v = v_cr
                s = s + v*dt
                t = t + dt
                
                indnew = indnew + 1                          
                
                #This part is made for the fact that when you brake, you take longer to cover distance, therefore extra taxi time is induced
                if indnew<=ind:
                    varray[indnew]=v
                    sarray[indnew]=s
                    tarray[indnew]=t
                    aarray[indnew]=a
                if indnew>ind:
                    tarray = np.append(tarray,t)
                    sarray = np.append(sarray,s)
                    varray = np.append(varray,v)
                    aarray = np.append(aarray,a)
            
            ind = indnew                                    #Correction for extra time
        
        #print('End velocity is', varray[ind], 'and end time is', tarray[ind])
             
    if taxiwayid[i]=='cr':                          #If we have a corner
        
        #print('The ',i,'th part is a corner')
        indstart = ind                              #Starting index in while loop
        
        v = varray[indstart]                        #Starting velocity in the turn
        
        while s<(taxiway[0][i]+sarray[indstart]):
            
            if v >= v_cr:                           #If v >= v_cr than constant velocity in turn
                v = v_cr
                s = s + v*dt
                a = 0
            if v < v_cr:                            #If velocity is slower than v_cr, room to accellerate in turn 
                a = max_a
                v = v + a*dt                        #For simplicity we say we will accellerate with maximum accelleration 
                s = s + v*dt
                
            t = t + dt
            ind = ind + 1
            
            tarray = np.append(tarray,t)
            sarray = np.append(sarray,s)
            varray = np.append(varray,v)
            aarray = np.append(aarray,a)
            
        #print('End velocity is', varray[ind], 'and end time is', tarray[ind])

plt.figure()    
plt.plot(tarray,varray)
plt.show()
#
#plt.figure()
#plt.plot(tarray,sarray)
#plt.show()
#
#plt.figure()
#plt.plot(tarray,aarray)
#plt.show()