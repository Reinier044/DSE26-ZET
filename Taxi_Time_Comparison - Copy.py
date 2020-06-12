"""
Created on Wed May 13 08:40:14 2020
@author: Julian
"""

import numpy as np
import matplotlib.pyplot as plt

# -------------------Input data CONV-system-----------------

#a_CONV = np.array([x1,x2,x3])
#v_CONV = np.array([0,7,12,14.97])
MTOW = 97400                    #Max Takeoff weight
Tfull = 2*155687.757            #Full thrust LEAP 1-A
t_to_full = 8                   #time to full thrust (engine spool)
dF_dt = Tfull//t_to_full        #Thrust increase
thrustsetting = 0.24            #Thrust setting applied
dt = 0.01                       #
max_v_eng = 15.433                  #Maximum achievable velocity achieved for A321 -> 30 kts [m/s]

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
    print(Responsetime)
    print(Responsevelocity)
    
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
    
print("Average acceleration: ",sum(a_CONV_part)/len(a_CONV_part))    

plt.figure()
plt.subplot(211)
plt.plot(ta_CONV_part, a_CONV_part)
plt.xlabel('Time')
plt.ylabel('acceleration')
plt.subplot(212)
plt.plot(ta_CONV_part, v_CONV_part)
plt.xlabel('Time')
plt.ylabel('velocity')
plt.show()

    
#a_CONV = np.array([da_dt,])               #Acceleration array achieved by CONV-system [m/s^2]
v_CONV = np.array([0, 1.8, 3.6, 5.22, 6.64, 7.80, 8.81, 9.73, 10.56, 11.33, 12.05, 12.72, 13.32, 13.89, 14.44, 14.97])   #Velocity array for acceleration CONV-system [m/s]
d_CONV = -0.7                    #Maximum deceleration achieved by CONV-system -> should be negative value! [m/s^2]

# -------------------Engine based taxiing--------------
a_eng = 0.7                         #Acceleration engine based taxiing [m/s^2]
max_d_eng = -0.7                    #Maximum deceleration conventional taxiing-> should be negative value! [m/s^2]
max_v_eng = 15.433                  #Maximum achievable velocity achieved for A321 -> 30 kts [m/s]

# --------------------Taxiway-------------------

# First row is distance of straight part or corner
# If corner, second row gives max velocity in turn-> 10; 15 or 20 [kts]

# Taxiway from D14 to runway 36C
#taxiway = np.array([[21.33, 35.52, 31.68, 43.17, 105.66, 60.91, 1383, 120, 950, 80, 60],
#                    [0, 5.1444, 0, 5.1444, 0, 5.1444, 0, 10.2889, 0, 5.1444, 5.1444]])

# Taxiway from D14 to Polderbaan
taxiway = np.array([[21.33,35.52,31.68,43.17,105.66,60.91,1383,120,754,140,280,70,210,40,130,160,2140,130,1690,150,360],
                    [0,5.1444,0,5.1444,0,5.1444,0,10.2889,0,5.1444,0,10.2889,0,7.7167,0,5.1444,0,7.7167,0,5.1444,0]])

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

        print('The ', i, 'th part is a straight part')
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
            print('Velocity next turn is', v_cr)

        if v > v_cr:            # If velocity at end of straight part is too high

            # Braking
            t_braking = (v_cr - v) / d_CONV
            print('Is the time for braking,', t_braking, ', reasonable for velocity',v,'?')

            indnew = ind - int(round(t_braking / dt))  # Go back in time-> this is the index where we will start braking

            if indnew < 0:  # Added after performing verification for high accelerations
                indnew = 0

            v = varray[indnew]  # Starting value in this while loop
            s = sarray[indnew]  # Starting value in this while loop
            t = tarray[indnew]  # Starting value in this while loop

            while sarray[indnew] <= taxiway[0][i] + sarray[indstart]:  # Of course we still need to cover all distances

    #v = v + a * dt
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

        print('End velocity is', varray[ind], 'and end time is', tarray[ind])

    if taxiwayid[i] == 'cr':  # If we have a corner

        print('The ', i, 'th part is a corner')
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

        print('End velocity is', varray[ind], 'and end time is', tarray[ind])


plt.figure()
plt.subplot(311)
plt.plot(tarray, varray)
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.subplot(312)
plt.plot(tarray, sarray)
plt.xlabel('Time')
plt.ylabel('Distance')
plt.subplot(313)
plt.plot(tarray, aarray)
plt.xlabel('Time')
plt.ylabel('Acceleration')
plt.show()


