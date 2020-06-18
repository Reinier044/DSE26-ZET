"""
Created on Wed May 13 08:40:14 2020
@author: Julian
"""

import numpy as np
import matplotlib.pyplot as plt
from Performance_Data import a_ZET,v_ZET,Pa_ZET,Pv_ZET,d_ZET,a_eng,max_d_eng,max_v_eng,dt,thrustsetting,power_cp
from Pushback import taxiway, taxiwayid, V_fin_segment, choose_2,choose
#define route "long" for limit case, "short" for performance check.


# --------------------Code-ZET----------------------
print('Calculating ZET taxi -',choose_2,choose,'route.')
# Simulation parameters
t = 0               # Starting time

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
status_array = np.array([taxiwayid[0]])

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
            status_array = np.append(status_array,taxiwayid[i])

        if i == len(taxiwayid)-1:                 # This is the last part, so no upcoming turn
            v_cr = V_fin_segment                          # Last straight part, make sure max 10 kts, change?
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
                    status_array = np.append(status_array,taxiwayid[i])

            ind = indnew  # Correction for extra time

        
    if taxiwayid[i] == 'cr':  # If we have a corner
        indstart = ind  # Starting index in while loop

        v = varray[indstart]  # Starting velocity in the turn

        if i == len(taxiwayid)-1:                 # This is the last part, so no upcoming turn
            v_cr = V_fin_segment                          # Last straight part, make sure max 10 kts, change?
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
            status_array = np.append(status_array,taxiwayid[i])
       
    
       
if choose_2 == 'out':     
    print('Adding pushback and coupling for ZET')
    #Implement pushback phase at beginning.
    from Pushback import tarray_pb,aarray_pb,varray_pb,sarray_pb,status_array_pb
    from Pushback import tarray_cp_ZET,aarray_cp_ZET,varray_cp_ZET,sarray_cp_ZET,status_array_cp_ZET
    
    
    tarray = np.append(tarray_pb,(tarray+tarray_pb[-1]))
    tarray = np.append(tarray,(tarray[-1]+tarray_cp_ZET))
    aarray = np.append(aarray_pb,aarray)
    aarray = np.append(aarray,aarray_cp_ZET)
    varray = np.append(-varray_pb,varray)
    varray = np.append(varray,varray_cp_ZET)
    sarray = np.append(-sarray_pb[::-1],sarray)
    sarray = np.append(sarray,(sarray_cp_ZET+sarray[-1]))
    status_array = np.append(status_array_pb,status_array)
    status_array = np.append(status_array,status_array_cp_ZET)
    
elif choose_2 == 'in':
    print('Adding coupling for ZET')
    #Implement pushback phase at beginning.
    from Pushback import tarray_cp_ZET,aarray_cp_ZET,varray_cp_ZET,sarray_cp_ZET,status_array_cp_ZET
    from Pushback import tarray_fin,varray_fin,sarray_fin,aarray_fin,status_array_fin
    tarray = np.append(tarray_cp_ZET,tarray+tarray_cp_ZET[-1])
    aarray = np.append(aarray_cp_ZET,aarray)
    varray = np.append(varray_cp_ZET,varray)
    sarray = np.append(sarray_cp_ZET,sarray)
    tarray = np.append(tarray,(tarray[-1]+tarray_fin))
    aarray = np.append(aarray,aarray_fin)
    varray = np.append(varray,varray_fin)
    sarray = np.append(sarray,(sarray[-1]+sarray_fin))
    status_array = np.append(status_array_cp_ZET,status_array)
    status_array = np.append(status_array,status_array_fin)
    
    

print('Calculating power and energy')
#Calculate the power for each moment in time
flag = 0       
while flag < len(tarray):
    if status_array[flag]!='cp':

        idx = 0
        while idx < len(v_ZET):
            if abs(varray[flag])>=v_ZET[idx]:
                if idx==(len(v_ZET)-1):
                    p = Pv_ZET[idx]
                    idx = len(v_ZET)+1
                elif abs(varray[flag])< v_ZET[idx+1]:
                    #Power required for acceleration
                    if aarray[flag]>0:
                        p = Pa_ZET[(idx)]
                        idx = len(v_ZET)+1
                    #Power required for constant velocity
                    elif aarray[flag]==0:
                        p = Pv_ZET[idx]
                        idx = len(v_ZET)+1
                    #No power required for braking
                    elif aarray[flag]<0:
                        p = 0
                        idx = len(v_ZET)+1
                else:
                    idx = idx + 1
            else:
                if idx<(len(v_ZET)-1):
                    idx = idx + 1
    elif status_array[flag]=='cp':
        p = power_cp
    
    #Calculate cumulative energy and store data. Power in [kW], energy in [J]
    e = e + (p*1000)*dt
    parray = np.append(parray, p)
    earray = np.append(earray, e)
    flag = flag + 1

#parray = np.append(parray,parray_cp_ZET)
#earray = np.append(earray,(earray_cp_ZET+earray[-1]))

ZETtarray = tarray
ZETvarray = varray
ZETsarray = sarray
ZETaarray = aarray



# -------------------Input data CONV-system-----------------
print('Calculating CONV taxi-',choose_2,',',choose,' route.')

MTOW = 97400                    #Max Takeoff weight
Tfull = 2*155687.757            #Full thrust LEAP 1-A
t_to_full = 8                   #time to full thrust (engine spool)
dF_dt = Tfull//t_to_full        #Thrust increase


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
      
v_CONV = v_ZET
d_CONV = max_d_eng                   #Maximum deceleration achieved by CONV-system -> should be negative value! [m/s^2]

# --------------------Code----------------------

# Simulation parameters
t = 0               # Starting time

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
            v_cr = V_fin_segment                        # Last straight part, make sure max 10 kts, change?
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
                            a = a_eng
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
            v_cr = V_fin_segment                         # Last straight part, make sure max 10 kts, change?
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
                        a = a_CONV_part[j+1]
                        break

                v = v + a * dt
                s = s + v * dt

            t = t + dt
            ind = ind + 1

            tarray = np.append(tarray, t)
            sarray = np.append(sarray, s)
            varray = np.append(varray, v)
            aarray = np.append(aarray, a)

if choose_2 == 'out': 
    print('Adding pushback and coupling for CONV (taxi out)')
    #Implement pushback phase at beginning.
    from Pushback import tarray_pb,aarray_pb,varray_pb,sarray_pb
    from Pushback import tarray_pb_CONV,aarray_pb_CONV,varray_pb_CONV,sarray_pb_CONV,tarray_fin,varray_fin,sarray_fin,aarray_fin  
    
    tarray = np.append(tarray_pb_CONV,tarray+tarray_pb_CONV[-1])
    tarray = np.append(tarray,(tarray_fin+tarray[-1]))
    aarray = np.append(-aarray_pb_CONV,aarray)
    aarray = np.append(aarray,aarray_fin)
    varray = np.append(-varray_pb_CONV,varray)
    varray = np.append(varray,varray_fin)
    sarray = np.append(sarray_pb_CONV,sarray)
    sarray = np.append(sarray,(sarray_fin+sarray[-1]))

#For taxi in, only add the braking segment to go to standstill at the gate
elif choose_2 == 'in':
    from Pushback import tarray_fin,varray_fin,sarray_fin,aarray_fin 
    tarray = np.append(tarray,(tarray[-1]+tarray_fin))
    aarray = np.append(aarray,aarray_fin)
    varray = np.append(varray,varray_fin)
    sarray = np.append(sarray,(sarray[-1]+sarray_fin))
    
print('creating figures')   

#make graphs 
plt.figure()
plt.subplot(211)
plt.plot(ZETtarray, parray)
plt.plot(ZETtarray, ZETvarray*70)
plt.xlabel('Time [s]')
plt.ylabel('Power [kW]')
plt.subplot(212)
plt.plot(ZETtarray, earray)
plt.xlabel('Time [s]')
plt.ylabel('energy [J]')
plt.show()

fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('time (s)')
ax1.set_ylabel('power', color=color)
ax1.plot(ZETtarray, parray, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:blue'
ax2.set_ylabel('velocity (m/s)', color=color)  # we already handled the x-label with ax1
ax2.plot(ZETtarray, ZETvarray, color=color)
ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()

         
plt.figure()
plt.subplot(311)
plt.plot(ZETtarray,ZETvarray, label = "ZET")
plt.plot(tarray, varray, label = "Conventional")
plt.xlabel('Time [s]')
plt.ylabel('Velocity [m/s]')
plt.legend()
plt.subplot(312)
plt.plot(ZETtarray,ZETsarray, label = "ZET")
plt.plot(tarray, sarray,label = "Conventional")
plt.xlabel('Time [s]')
plt.ylabel('Distance [m]')
plt.legend()
plt.subplot(313)
plt.plot(ZETtarray,ZETaarray, label = "ZET")
plt.plot(tarray, aarray, label = "Conventional")
plt.xlabel('Time [s]')
plt.ylabel('Acceleration [m/s^2]')
plt.legend()
plt.show()



if ZETtarray[-1]> tarray[-1]:
    print("ZET is ", ZETtarray[-1]-tarray[-1], "seconds slower")
if ZETtarray[-1]<tarray[-1]:
    print("ZET is ", tarray[-1]-ZETtarray[-1], "seconds faster")
 

#tot_a = 0
#tot_t = 0
#for accel in ZETaarray:
#    if accel>0:
#        tot_a = tot_a + accel
#        tot_t = tot_t + 1

#print('average acceleration ZET:', tot_a/tot_t)
#print('avg speed ZET:',sum(ZETvarray)/len(ZETvarray))

#tot_a = 0
#tot_t = 0
#for accel in aarray:
#    if accel>0:
#        tot_a = tot_a + accel
#        tot_t = tot_t + 1
#    
#print('average acceleration CONV:', tot_a/tot_t)
#print('avg speed CONV:', sum(varray)/len(varray))
print('Total energy of cycle: ', earray[-1] ,'Joules')