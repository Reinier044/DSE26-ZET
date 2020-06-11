"""
Created on Wed May 13 08:40:14 2020

@author: Julian
"""

import numpy as np
import matplotlib.pyplot as plt

# -------------------Input data ZET-system-----------------

a_ZET = np.array([1.8, 1.8, 1.62, 1.42, 1.16, 1.01, 0.92, 0.83, 0.77, 0.72, 0.67, 0.6, 0.57, 0.55, 0.53])               #Acceleration array achieved by ZET-system [m/s^2]
v_ZET = np.array([0, 1.8, 3.6, 5.22, 6.64, 7.80, 8.81, 9.73, 10.56, 11.33, 12.05, 12.72, 13.32, 13.89, 14.44, 14.97])   #Velocity array for acceleration ZET-system [m/s]
Pa_ZET = np.array([0.0, 53503.79505171736, 107007.59010343472, 140972.0799287942, 159266.6417338863, 156465.4477782152, 156769.70906008425, 159916.69614760645, 159206.08300577733, 160549.18119295375, 161653.4077472227, 161037.3518413722, 154553.25502229988, 154874.42215552504, 156645.78184122857, 157873.99840976746])
Pv_ZET = np.array([0.0, 4576.464978661564, 9152.929957323128, 13271.748438118538, 16882.07081017377, 19831.348240866777, 22399.253590004655, 24738.335690209453, 26848.594541481173, 28806.30456013084, 36368.90551595467, 32340.35251587505, 338658.4084209557, 35315.05475200507, 36713.419051040546, 38060.933739202])
d_ZET = -0.7                    #Maximum deceleration achieved by ZET-system -> should be negative value! [m/s^2]

v_cr = 5.144 * 1.5              #Limit on speed on turns (approx 10 kts -> 15 kts) [m/s]

# -------------------Engine based taxiing--------------
a_eng = 0.7                         #Acceleration engine based taxiing [m/s^2]
max_d_eng = -0.7                    #Maximum deceleration conventional taxiing-> should be negative value! [m/s^2]
max_v_eng = 15.433                  #Maximum achievable velocity achieved for A321 -> 30 kts [m/s]
Pmax_v = Pv_ZET[-1]                 #Power at maximum velocity

# --------------------Taxiway-------------------

# First row is distance of straight part or corner
# If corner, second row gives turn radius -> otherwise 0
# In this code, the second row in the array is never used. However might be useful to make it more accurate

# Taxiway from D14 to runway 36C
# taxiway = np.array([[21.33, 35.52, 31.68, 43.17, 105.66, 60.91, 1383, 120, 950, 80, 60],
#                    [0, 38.8, 0, 44.8, 0, 49.5, 0, 105, 0, 43.6, 52.6]])

# Taxiway from D14 to Polderbaan
taxiway = np.array([[21.33,35.52,31.68,43.17,105.66,60.91,1383,120,754,140,893,70,210,40,130,160,2140,130,1690,150,360],
                    [0,38.8,0,44.8,0,49.5,0,105,0,75.8,0,90.6,0,94,0,101.5,0,172,0,107.5,0]])

# Taxiwayid show whether we have straight part (st) or corner (cr)

# Taxiway ID from D14 to runway 36C
#taxiwayid = np.array(['st', 'cr', 'st', 'cr', 'st', 'cr', 'st', 'cr', 'st', 'cr', 'cr'])

# Taxiway ID from D14 to Polderbaan
taxiwayid = np.array(['st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st'])

# --------------------Code----------------------

# Simulation parameters
t = 0               # Starting time
dt = 0.01           # Time step

# Initial conditions
v = 0               # Starting velocity
s = 0               # Starting distance
e = 0               # Starting energy
ind = 0             # Index value

# Storing arrays
tarray = np.array([0])
sarray = np.array([0])
varray = np.array([0.0000001])
aarray = np.array([a_ZET[0]])
parray = np.array([0])
earray = np.array([0])

# Simulation
# Only works when you start with straight distance, otherwise we need small modification

for i in range(len(taxiwayid)):

    if taxiwayid[i] == 'st':                                    # If we have straight part

        print('The ', i, 'th part is a straight part')
        indstart = ind                                          # Starting index in while loop
        v = varray[indstart]                                    # Starting velocity in straight part

        while s < (taxiway[0][i] + sarray[indstart]):

            for j in range(len(v_ZET)):
                if v>v_ZET[j] and v<=v_ZET[j+1]:
                    a = a_ZET[j]
                    p = Pa_ZET[j]
                    break
            

            v = v + a * dt
            if v > v_ZET[-1]:                                   # It can never exceed maximum speed
                v = v_ZET[-1]
                a = 0                                           # If maximum speed is achieved, a = 0
                #find which index belongs to the speed
                for spdid in range(len(v_ZET)):
                    if v_ZET[spdid] == v:
                        p =Pv_ZET[spdid]
                    
                        
                
            e = e + p * dt
            s = s + v * dt
            t = t + dt
            ind = ind + 1

            # Appending values to arrays
            tarray = np.append(tarray, t)
            sarray = np.append(sarray, s)
            varray = np.append(varray, v)
            aarray = np.append(aarray, a)
            parray = np.append(parray, p)
            earray = np.append(earray, e)

        if v > v_cr:            # If velocity at end of straight part is too high

            # Braking
            t_braking = (v_cr - v) / d_ZET
            #print('Is the time for braking,', t_braking, ', reasonable for velocity',v,'?')

            indnew = ind - int(round(t_braking / dt))  # Go back in time-> this is the index where we will start braking

            if indnew < 0:  # Added after performing verification for high accelerations
                indnew = 0

            a = d_ZET           # Use maximum deceleration for braking

            v = varray[indnew]  # Starting value in this while loop
            s = sarray[indnew]  # Starting value in this while loop
            t = tarray[indnew]  # Starting value in this while loop
            p = parray[indnew]  # Starting value in this while loop
            e = earray[indnew]  # Starting value in this while loop

            while sarray[indnew] <= (taxiway[0][i] + sarray[indstart]):  # Of course we still need to cover all distances

                #v = v + a * dt
                if v < v_cr:  # For now, as first order estimate, if we brake sufficiently a small part has a velocity of v_cr before turn

                    for j in range(len(v_ZET)):
                        if v > v_ZET[j] and v <= v_ZET[j + 1]:
                            a = a_ZET[j]
                            p = Pa_ZET[j]
                            break

                    v = v + a * dt
                    

                    if v>v_cr:
                        v = v_cr

                if v == v_cr:
                    v = v_cr
                    a = 0
                    for spdid in range(len(v_ZET)):
                        if v_ZET[spdid] >= v:
                            p = Pv_ZET[spdid]
                            break
                        
                if v > v_cr:
                    a = d_ZET
                    v = v + a * dt
                    

                    if v<v_cr:
                        v = v_cr

                s = s + v * dt
                t = t + dt
                e = e + p*dt
                p = 0

                indnew = indnew + 1

                # This part is made for the fact that when you brake, you take longer to cover distance, therefore extra taxi time is induced
                if indnew <= ind:
                    varray[indnew] = v
                    sarray[indnew] = s
                    tarray[indnew] = t
                    aarray[indnew] = a
                    parray[indnew] = p
                    earray[indnew] = e
                if indnew > ind:
                    tarray = np.append(tarray, t)
                    sarray = np.append(sarray, s)
                    varray = np.append(varray, v)
                    aarray = np.append(aarray, a)
                    parray = np.append(parray, p)
                    earray = np.append(earray, e)

            ind = indnew  # Correction for extra time

        #print('End velocity is', varray[ind], 'and end time is', tarray[ind])

    elif taxiwayid[i] == 'cr':  # If we have a corner

        print('The ', i, 'th part is a corner')
        indstart = ind  # Starting index in while loop

        v = varray[indstart]  # Starting velocity in the turn

        while s < (taxiway[0][i] + sarray[indstart]):
            
            if v >= v_cr:  # If v >= v_cr than constant velocity in turn
                v = v_cr
                s = s + v * dt
                a = 0
                for spdid in range(len(v_ZET)):
                    if v_ZET[spdid] > v:
                        p = Pv_ZET[spdid]
                        break
               
            elif v < v_cr:  # If velocity is slower than v_cr, room to accellerate in turn
                for j in range(len(v_ZET)):
                    if v > v_ZET[j] and v <= v_ZET[j + 1]:
                        a = a_ZET[j]
                        p = Pa_ZET[j]
                        break

                        
                v = v + a * dt
                s = s + v * dt
            
            e = e + p * dt
            t = t + dt
            ind = ind + 1

            tarray = np.append(tarray, t)
            sarray = np.append(sarray, s)
            varray = np.append(varray, v)
            aarray = np.append(aarray, a)
            parray = np.append(parray, p)
            earray = np.append(earray, e)

        print('End velocity is', varray[ind], 'and end time is', tarray[ind])

# Plots also used for verification

#plt.figure()
#plt.plot(tarray, varray)
#plt.xlabel('Time')
#plt.ylabel('Velocity')
#plt.show()
#
#plt.figure()
#plt.plot(tarray, sarray)
#plt.xlabel('Time')
#plt.ylabel('Distance')
#plt.show()
#
#plt.figure()
#plt.plot(tarray, aarray)
#plt.xlabel('Time')
#plt.ylabel('Acceleration')
#plt.show()

plt.figure()
plt.subplot(211)
plt.plot(tarray, parray)
plt.xlabel('Time')
plt.ylabel('power')
plt.subplot(212)
plt.plot(tarray, varray)
plt.xlabel('Time')
plt.ylabel('velocity')
plt.show()

plt.figure()
plt.plot(tarray, earray)
plt.xlabel('Time')
plt.ylabel('energy')
plt.show()

