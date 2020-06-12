"""
Created on Wed May 13 08:40:14 2020
@author: Julian
"""

import numpy as np
import matplotlib.pyplot as plt
import sys

#define route "long" for limit case, "short" for performance check.
choose = "short" 

# -------------------Input data ZET-system-----------------
constant_a = 1.0
a_ZET = np.array([constant_a-0.00010,constant_a-0.0009,constant_a-0.0008,constant_a-0.0007,constant_a-0.0006,constant_a-0.0005,constant_a-0.0004,constant_a-0.0003,constant_a-0.0002,constant_a+0.0002,constant_a+0.0003,constant_a+0.0004,constant_a+0.0005,constant_a+0.0006,constant_a+0.0007,constant_a+0.0008,constant_a+0.0009])
v_ZET = np.array([0, 1.55, 3.1, 4.62, 6.04, 7.2, 8.21, 9.13, 9.96, 10.73, 11.45, 12.12, 12.72, 13.84, 14.37, 14.97, 15.433])
Pa_ZET = np.array([0.0, 74.97143255879, 14.994286511757, 22.491429767636, 29.988573023515, 37.485716279393, 44.982859535272, 52.480002791151, 59.977146047029, 67.474289302908,  74.971432558787,  82.468575814666,89.965719070544,  97.462862326423, 104.960005582302, 112.45714883818 , 119.954292094059])
Pv_ZET = np.array([  0.,  10.80360011,  21.60720021,  32.41080032, 43.21440042,  54.01800053,  64.82160064,  75.62520074, 86.42880085,  97.23240095, 108.03600106, 118.83960116, 129.64320127, 140.44680138, 151.25040148, 162.05400159, 172.85760169])
d_ZET = -0.7                    #Maximum deceleration achieved by ZET-system -> should be negative value! [m/s^2]
a_eng = 0.7                         #Acceleration engine based taxiing [m/s^2]
max_d_eng = -0.7                    #Maximum deceleration conventional taxiing-> should be negative value! [m/s^2]
max_v_eng = v_ZET[-1]               #Maximum achievable velocity achieved for A321 -> 30 kts [m/s]

# --------------------Taxiway-------------------

if choose == "long":
    #Taxiway from D14 to Polderbaan
    taxiway = np.array([[21.33,35.52,31.68,43.17,105.66,60.91,1383,120,754,140,280,70,210,40,130,160,2140,130,1690,150,360],
                    [0,5.1444,0,5.1444,0,5.1444,0,10.2889,0,5.1444,0,10.2889,0,7.7167,0,5.1444,0,7.7167,0,5.1444,0]])
    
    # Taxiway ID from D14 to Polderbaan
    taxiwayid = np.array(['st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st'])
# First row is distance of straight part or corner
# If corner, second row gives max velocity in turn-> 10; 15 or 20 [kts]
elif choose == "short":
    # Taxiway from D14 to runway 36C
    taxiway = np.array([[21.33, 35.52, 31.68, 43.17, 105.66, 60.91, 1383, 120, 950, 80, 60],
                    [0, 5.1444, 0, 5.1444, 0, 5.1444, 0, 10.2889, 0, 5.1444, 5.1444]])

    # Taxiway ID from D14 to runway 36C
    taxiwayid = np.array(['st', 'cr', 'st', 'cr', 'st', 'cr', 'st', 'cr', 'st', 'cr', 'cr'])
    # Taxiwayid show whether we have straight part (st) or corner (cr)

else:
    sys.exit("No valid taxi route.")


# --------------------Code-ZET----------------------

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
parray = np.array([])
earray = np.array([])

# Simulation
# Only works when you start with straight distance, otherwise we need small modification
for i in range(len(taxiwayid)):

    if taxiwayid[i] == 'st':                                    # If we have straight part

        #print('The ', i, 'th part is a straight part')
        indstart = ind                                          # Starting index in while loop
        v = varray[indstart]                                    # Starting velocity in straight part

        while s < taxiway[0][i] + sarray[indstart]:             #Needed distance covered [m]

            for j in range(len(v_ZET)):
                if v>v_ZET[j] and v<=v_ZET[j+1]:
                    a = a_ZET[j]
                    break

            v = v + a * dt

            if v > v_ZET[-1]:                                   # It can never exceed maximum speed
                v = v_ZET[-1]
                a = 0                                           # If maximum speed is achieved, a = 0
                
            s = s + v * dt
            t = t + dt
            ind = ind + 1

            # Appending values to arrays
            tarray = np.append(tarray, t)
            sarray = np.append(sarray, s)
            varray = np.append(varray, v)
            aarray = np.append(aarray, a)

        if i == len(taxiwayid)-1:                 # This is the last part, so no upcoming turn
            v_cr = 5.1444                         # Last straight part, make sure max 10 kts, change?
        else:                                     # After straight part, always a turn!
            v_cr = taxiway[1][i+1]                # Velocity in turn is dependant on turn coming

        if v > v_cr:            # If velocity at end of straight part is too high

            # Braking
            t_braking = (v_cr - v) / d_ZET

            indnew = ind - int(round(t_braking / dt))  # Go back in time-> this is the index where we will start braking

            if indnew < 0:  # Added after performing verification for high accelerations
                indnew = 0

            v = varray[indnew]  # Starting value in this while loop
            s = sarray[indnew]  # Starting value in this while loop
            t = tarray[indnew]  # Starting value in this while loop

            while sarray[indnew] <= taxiway[0][i] + sarray[indstart]:  # Of course we still need to cover all distances

                if v < v_cr:                #If velocity is still smaller than v_cr, room to accelerate to v_cr

                    for j in range(len(v_ZET)):                         #Find acceleration at which we can accelerate
                        if v > v_ZET[j] and v <= v_ZET[j + 1]:
                            a = a_ZET[j]
                            break

                    v = v + a * dt

                    if v>v_cr: # For now, as first order estimate, if we brake sufficiently a small part has a velocity of v_cr before turn
                        v = v_cr
                        a = 0

                if v > v_cr:
                    a = d_ZET
                    v = v + a * dt

                    if v<v_cr:
                        v = v_cr
                        a = 0

                if v == v_cr:
                    a = 0

                s = s + v * dt
                t = t + dt

                indnew = indnew + 1

                # This part is made for the fact that when you brake, you take longer to cover distance, therefore extra taxi time is induced
                if indnew <= ind:
                    varray[indnew] = v
                    sarray[indnew] = s
                    tarray[indnew] = t
                    aarray[indnew] = a
                if indnew > ind:
                    tarray = np.append(tarray, t)
                    sarray = np.append(sarray, s)
                    varray = np.append(varray, v)
                    aarray = np.append(aarray, a)

            ind = indnew  # Correction for extra time

    if taxiwayid[i] == 'cr':  # If we have a corner

        indstart = ind  # Starting index in while loop

        v = varray[indstart]  # Starting velocity in the turn

        if i == len(taxiwayid)-1:                 # This is the last part, so no upcoming turn
            v_cr = 5.1444                         # Last straight part, make sure max 10 kts, change?
        else:                                     # After straight part, always a turn!
            v_cr = taxiway[1][i]                  # Velocity in turn is dependant on turn coming

        while s < (taxiway[0][i] + sarray[indstart]):

            if v >= v_cr:  # If v >= v_cr than constant velocity in turn
                v = v_cr
                s = s + v * dt
                a = 0
            if v < v_cr:  # If velocity is slower than v_cr, room to accellerate in turn

                for j in range(len(v_ZET)):
                    if v > v_ZET[j] and v <= v_ZET[j + 1]:
                        a = a_ZET[j]
                        break

                v = v + a * dt
                s = s + v * dt

            t = t + dt
            ind = ind + 1

            tarray = np.append(tarray, t)
            sarray = np.append(sarray, s)
            varray = np.append(varray, v)
            aarray = np.append(aarray, a)


#Calculate the power for each moment in time
flag = 0       
while flag < len(tarray):
    
    #For positive acceleration, take value for the corresponding power
    if aarray[flag]>0:                      
        idx = 0
        while idx <len(a_ZET):
            if a_ZET[idx] == aarray[flag]:
                p = Pa_ZET[idx]
                idx = len(a_ZET)
            else:
                idx = idx + 1
     
    #For constant speed, take value for corresponding power (overcome drag)           
    if aarray[flag]==0:
        idx = 0
        while idx <len(v_ZET):
            if v_ZET[idx] > varray[flag]:
                p = Pv_ZET[idx]
                idx = len(v_ZET)
            else:
                idx = idx + 1
     
    #For breaking, no power input required           
    if aarray[flag]<0:
        p = 0
    
    #Calculate cumulative energy and store data.
    e = e + p*dt
    parray = np.append(parray, p)
    earray = np.append(earray, e)
    flag = flag + 1

plt.figure()
plt.subplot(211)
plt.plot(tarray, parray)
plt.plot(tarray, varray)
plt.xlabel('Time')
plt.ylabel('power')
plt.subplot(212)
plt.plot(tarray, earray)
plt.xlabel('Time')
plt.ylabel('energy')
plt.show()

ZETtarray = tarray
ZETvarray = varray
ZETsarray = sarray
ZETaarray = aarray


# -------------------Input data CONV-system-----------------

MTOW = 97400                    #Max Takeoff weight
Tfull = 2*155687.757            #Full thrust LEAP 1-A
t_to_full = 8                   #time to full thrust (engine spool)
dF_dt = Tfull//t_to_full        #Thrust increase
thrustsetting = 0.24            #Thrust setting applied


a_CONV_part = np.array([])
a_nonlin_part = np.array([])
ta_CONV_part = np.array([])
v_CONV_part = np.array([])
Thrust = 0
t= 0.0
velocity = 0
vmax = max_v_eng
if Thrust < thrustsetting:
    while Thrust < thrustsetting*Tfull:
        a_CONV_part = np.append(a_CONV_part, Thrust/MTOW)
        a_nonlin_part = np.append(a_nonlin_part, Thrust/MTOW)
        ta_CONV_part = np.append(ta_CONV_part, t)
        v_CONV_part = np.append(v_CONV_part, velocity)
        t = t+dt
        velocity = velocity + (Thrust/MTOW)*dt
        Thrust = Thrust+ dF_dt*dt  
    Responsetime = t
    Responsevelocity = velocity
    
while velocity < vmax-Responsevelocity:
    a_CONV_part = np.append(a_CONV_part, Thrust/MTOW)
    ta_CONV_part = np.append(ta_CONV_part, t)
    v_CONV_part = np.append(v_CONV_part, velocity)
    t = t+dt
    velocity = velocity + (Thrust/MTOW)*dt

a_nonlin_part = np.flip(a_nonlin_part)
for a in a_nonlin_part:
    a_CONV_part = np.append(a_CONV_part, a)
    ta_CONV_part = np.append(ta_CONV_part, t)
    v_CONV_part = np.append(v_CONV_part, velocity)
    t = t+dt
    velocity = velocity + a*dt       
      
v_CONV = np.array([0, 1.8, 3.6, 5.22, 6.64, 7.80, 8.81, 9.73, 10.56, 11.33, 12.05, 12.72, 13.32, 13.89, 14.44, 14.97])   #Velocity array for acceleration CONV-system [m/s]
d_CONV = -0.7                    #Maximum deceleration achieved by CONV-system -> should be negative value! [m/s^2]

# --------------------Code----------------------

# Simulation parameters
t = 0               # Starting time
dt = 0.01           # Time step

# Initial conditions
v = 0               # Starting velocity
s = 0               # Starting distance
ind = 0             # Index value

# Storing arrays
tarray = np.array([0])
sarray = np.array([0])
varray = np.array([0.0000001])
aarray = np.array([a_CONV_part[0]])

# Simulation
# Only works when you start with straight distance, otherwise we need small modification

for i in range(len(taxiwayid)):

    if taxiwayid[i] == 'st':                                    # If we have straight part

#        print('The ', i, 'th part is a straight part')
        indstart = ind                                          # Starting index in while loop
        v = varray[indstart]                                    # Starting velocity in straight part
        
        a_CONV_part = np.array([])
        ta_CONV_part = np.array([0])
        v_CONV_part = np.array([v])
        T_CONV_part = np.array([0])
        flag = 0
        
        while s < taxiway[0][i] + sarray[indstart]:             #Needed distance covered [m]


            Thrust = T_CONV_part[-1]
            t_part= ta_CONV_part[-1]
            
            if Thrust < thrustsetting*Tfull and v< max_v_eng-Responsevelocity:
                a = Thrust/MTOW
                a_CONV_part = np.append(a_CONV_part, a)
                v_CONV_part = np.append(v_CONV_part, v)
                t_part = t_part+dt
                Thrust = Thrust + dF_dt*dt
                T_CONV_part = np.append(T_CONV_part, Thrust)
                ta_CONV_part = np.append(ta_CONV_part, t_part)

                 
            elif v < max_v_eng-Responsevelocity:
                a = Thrust/MTOW
                a_CONV_part = np.append(a_CONV_part, a)
                ta_CONV_part = np.append(ta_CONV_part, t)
                v_CONV_part = np.append(v_CONV_part, v)
                t_part = t_part+dt

           
            elif v < max_v_eng:
                a = a_nonlin_part[flag]
                a_CONV_part = np.append(a_CONV_part, a)
                ta_CONV_part = np.append(ta_CONV_part, t)
                v_CONV_part = np.append(v_CONV_part, v)
                t_part = t_part+dt  
                flag = flag + 1
                
            if v > max_v_eng:                                   # It can never exceed maximum speed
                v = max_v_eng
                a = 0                                           # If maximum speed is achieved, a = 0

            v = v + a * dt
            s = s + v * dt
            t = t + dt
            ind = ind + 1

            # Appending values to arrays
            tarray = np.append(tarray, t)
            sarray = np.append(sarray, s)
            varray = np.append(varray, v)
            aarray = np.append(aarray, a)

        if i == len(taxiwayid)-1:                 # This is the last part, so no upcoming turn
            v_cr = 5.1444                         # Last straight part, make sure max 10 kts, change?
        else:                                     # After straight part, always a turn!
            v_cr = taxiway[1][i+1]                # Velocity in turn is dependant on turn coming

        if v > v_cr:            # If velocity at end of straight part is too high

            # Braking
            t_braking = (v_cr - v) / d_CONV

            indnew = ind - int(round(t_braking / dt))  # Go back in time-> this is the index where we will start braking

            if indnew < 0:  # Added after performing verification for high accelerations
                indnew = 0

            v = varray[indnew]  # Starting value in this while loop
            s = sarray[indnew]  # Starting value in this while loop
            t = tarray[indnew]  # Starting value in this while loop

            while sarray[indnew] <= taxiway[0][i] + sarray[indstart]:  # Of course we still need to cover all distances

                if v < v_cr:                #If velocity is still smaller than v_cr, room to accelerate to v_cr

                    for j in range(len(v_CONV)):                         #Find acceleration at which we can accelerate
                        if v > v_CONV[j] and v <= v_CONV[j + 1]:
                            a = 0.771227
                            break

                    v = v + a * dt

                    if v>v_cr: # For now, as first order estimate, if we brake sufficiently a small part has a velocity of v_cr before turn
                        v = v_cr
                        a = 0

                if v > v_cr:
                    a = d_CONV
                    v = v + a * dt

                    if v<v_cr:
                        v = v_cr
                        a = 0

                if v == v_cr:
                    a = 0

                s = s + v * dt
                t = t + dt

                indnew = indnew + 1

                # This part is made for the fact that when you brake, you take longer to cover distance, therefore extra taxi time is induced
                if indnew <= ind:
                    varray[indnew] = v
                    sarray[indnew] = s
                    tarray[indnew] = t
                    aarray[indnew] = a
                if indnew > ind:
                    tarray = np.append(tarray, t)
                    sarray = np.append(sarray, s)
                    varray = np.append(varray, v)
                    aarray = np.append(aarray, a)

            ind = indnew  # Correction for extra time

    if taxiwayid[i] == 'cr':  # If we have a corner

#        print('The ', i, 'th part is a corner')
        indstart = ind  # Starting index in while loop

        v = varray[indstart]  # Starting velocity in the turn

        if i == len(taxiwayid)-1:                 # This is the last part, so no upcoming turn
            v_cr = 5.1444                         # Last straight part, make sure max 10 kts, change?
        else:                                     # After straight part, always a turn!
            v_cr = taxiway[1][i]                  # Velocity in turn is dependant on turn coming

        while s < (taxiway[0][i] + sarray[indstart]):

            if v >= v_cr:  # If v >= v_cr than constant velocity in turn
                v = v_cr
                s = s + v * dt
                a = 0
            if v < v_cr:  # If velocity is slower than v_cr, room to accellerate in turn

                for j in range(len(v_CONV)):
                    if v > v_CONV[j] and v <= v_CONV[j + 1]:
                        a = a_CONV_part[j]
                        break

                v = v + a * dt
                s = s + v * dt

            t = t + dt
            ind = ind + 1

            tarray = np.append(tarray, t)
            sarray = np.append(sarray, s)
            varray = np.append(varray, v)
            aarray = np.append(aarray, a)


# make graphs          
plt.figure()
plt.subplot(311)
plt.plot(ZETtarray,ZETvarray, label = "ZET")
plt.plot(tarray, varray, label = "Conventional")
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.subplot(312)
plt.plot(ZETtarray,ZETsarray, label = "ZET")
plt.plot(tarray, sarray,label = "Conventional")
plt.xlabel('Time')
plt.ylabel('Distance')
plt.subplot(313)
plt.plot(ZETtarray,ZETaarray, label = "ZET")
plt.plot(tarray, aarray, label = "Conventional")
plt.xlabel('Time')
plt.ylabel('Acceleration')
plt.legend()
plt.show()

if ZETtarray[-1]> tarray[-1]:
    print("ZET is ", ZETtarray[-1]-tarray[-1], "seconds slower")
if ZETtarray[-1]<tarray[-1]:
    print("ZET is ", tarray[-1]-ZETtarray[-1], "seconds faster")
