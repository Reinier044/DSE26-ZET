"""
Created on Wed May 13 08:40:14 2020

@author: Julian
"""

import numpy as np
import matplotlib.pyplot as plt

# -------------------Input data ZET-system-----------------

a_ZET = np.array([1.8, 1.8, 1.62, 1.42, 1.16, 1.01, 0.92, 0.83, 0.77, 0.72, 0.67, 0.6, 0.57, 0.55, 0.53])               #Acceleration array achieved by ZET-system [m/s^2]
v_ZET = np.array([0, 1.8, 3.6, 5.22, 6.64, 7.80, 8.81, 9.73, 10.56, 11.33, 12.05, 12.72, 13.32, 13.89, 14.44, 14.97])   #Velocity array for acceleration ZET-system [m/s]
d_ZET = -0.7                    #Maximum deceleration achieved by ZET-system -> should be negative value! [m/s^2]

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
aarray = np.array([a_ZET[0]])

# Simulation
# Only works when you start with straight distance, otherwise we need small modification

for i in range(len(taxiwayid)):

    if taxiwayid[i] == 'st':                                    # If we have straight part

        print('The ', i, 'th part is a straight part')
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
            print('Velocity next turn is', v_cr)

        if v > v_cr:            # If velocity at end of straight part is too high

            # Braking
            t_braking = (v_cr - v) / d_ZET
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

        print('End velocity is', varray[ind], 'and end time is', tarray[ind])

# Plots also used for verification

plt.figure()
plt.plot(tarray, varray)
plt.xlabel('Time')
plt.ylabel('Velocity')
plt.show()

plt.figure()
plt.plot(tarray, sarray)
plt.xlabel('Time')
plt.ylabel('Distance')
plt.show()

plt.figure()
plt.plot(tarray, aarray)
plt.xlabel('Time')
plt.ylabel('Acceleration')
plt.show()
