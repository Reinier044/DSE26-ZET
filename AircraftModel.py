# -*- coding: utf-8 -*-
"""
Created on Wed May 13 10:14:36 2020
Created on Wed May 13 14:53:58 2020

@author: Reinier
@author: stijn
"""
import numpy as np

#Size data
NGx         = 5.07      #[m] Position of NG on x-axis
MLGx        = 21.97     #[m] Position of MLG on x-axis
MACcg       = 0.3607    #[%] cg position
XLEMAC      = 19.611397 #[m] X Position mean aerodynamic chord
MAC         = 4.29      #[m] Length mean aerodynamic chord
Hfslg       = 6.01      #[m] Fuselage height
Dfslg       = 4.14      #[m] Fuselage diameter
Rmlg        = 0.635     #[m] diameter MLG
MRW         = 97400     #[kg] Maximum ramp weight
g           = 9.81      #[m/s^2] Gravity 


#Variables
DesAccel        = 1.3           #[m/s^2]Desired acceleration
Rvw             = 0.25          #[m] Radius Vehicle Wheel
MLGTugW         = 15000         #[kg] estimated tug weight
NGTugW          = 15000         #[kg] estimated nose gear tug weight
Slope           = 0.0           #[rad] 0.0300196631 max (aka 3%)
CGfslg          = 1/3           #[%] assumed position of cg wrt fuselage (0 is bottom, 1 is top)
LiftNG          = 0.05          #[m] How high the nosegear is lifted
LiftMLG         = 0.05          #[m] How high the main landing gear is lifted
MuRolStatDry    = 0.02          #[-]Static friction coefficient dry surface
MuRolDynDry     = 0.02          #[-]Dynamic friction coefficient dry surface
MuKinDry        = 0.8           #[-]range of 0.6 - 0.85
MuKinWet        = 0.5           #[-]range of 0.45 - 0.75
NumEngines      = 3             #Number of engines
NumWheels       = 6             #Number of wheels on driving axles
GearRatio       = 14.95         #Gear ratio
max_d           = -1.5          #Maximum deceleration -> should be negative valu5.144e!
max_v           = 12.861        #Maximum achievable velocity -> 30 kts (15.433), 25 kts (12.86)
v_cr            = 5.144         #Limit on speed on turns (approx 10 kts)
std_taxtime     = 844.739       #time it takes normal taxi operations (25kts) to reach polderbaan. Pushback excluded
Taxi_with_ac    = "no"         #Taxi with aircraft attached (yes) or not (anything else)
MaxPower        = 250           #[kW] Selected engine MaxPower
EngineRPM       = 1491          #RPM for selected engine


#Calculate cg position
Liftslope   = np.arctan((LiftMLG-LiftNG)/(MLGx-NGx))
cgx         = MACcg*MAC+XLEMAC                                                  #Position cg in x-direction, from nose to tail
cgy         = CGfslg*Dfslg+Hfslg-Dfslg+(LiftNG+(np.tan(Liftslope)*(cgx-NGx)))   #Position cg in y-direction, from ground up
cgz         = 0                                                                 #Position cg in z-direction, from center fuselage to left wing

if Taxi_with_ac == "yes":
    MRW = MRW + 2*MLGTugW + NGTugW  #weight of combinationo 
else:
    MRW = 2*MLGTugW + NGTugW
    MaxPower = 0.5*MaxPower
    

#Calculate forces for limit thrust
ThrustArm       = cgy - Rmlg  #[m]Arm for thrust application (y-direction)
ResThrust       = MRW*DesAccel  #[N] Resultant force for the desired acceleration                                            
Weightx         = MRW*g*np.sin(Slope) #[N] Weight in x direction (due to slope)
Weighty         = MRW*g*np.cos(Slope) #[N] Weight in y direction (due to slope)
MLGnormalstat   = (Weightx*(ThrustArm)+(Weighty*(cgx-NGx)))/((cgx-NGx)+(MLGx-cgx))  #[N] MLG Normal force in static situation. Verified with airport planing manual
NGnormalstat    = Weighty-MLGnormalstat                                             #[N] NG Normal force in static situation. Verified with airport planing manual
NGnormal        = 0                                                                 #[N] Main landing gear normal force at tipover. 0 by definition
MLGnormal       = Weighty                                                           #[N] Main landing gear normal force at tipover. 
MaxThrust       = (MLGnormal*(MLGx-cgx) - NGnormal*(cgx-NGx))/ThrustArm             #[N]Maximum Force before tipover (+ DragNG*cgy + DragMLG*cgy)

#Calculate normal Forces
NGnormal    = NGnormalstat-(NGnormalstat/MaxThrust)*ResThrust   #[N] NG Normal force for max acceleration.
MLGnormal   = Weighty-NGnormal                                  #[N] MLG Normal force for max acceleration

#Calculate drag
FrolStat    = MuRolStatDry*(MLGnormal+NGnormal) #[N] Static friction force
DragNG      = MuRolDynDry*NGnormal              #[N] Dynamic friction MLG
DragMLG     = MuRolDynDry*MLGnormal             #[N] Dynamic friction NG
DragRoll    = DragNG + DragMLG                  #[N] Total drag during roll
ThrustSetMax= ResThrust+Weightx+DragRoll        #[N] Max thrust to be set

#Calculate min and max force and torques
MinTorque = (FrolStat/NumWheels) *Rvw          #[N*m] Minimal torque per wheel
Torque = (ThrustSetMax/NumWheels) *Rvw         #[N*m] Torque per wheel to get resultant force
Tmax_axle = (NumWheels/NumEngines)*Torque      #[N*m] Max torque per engine to get resultant force
Tmin_axle = (NumWheels/NumEngines)*MinTorque   #[N*m] Min torque per engine

#Calculate gearing ratio's required
GearTopSpeed = EngineRPM/((max_v/(2*np.pi*Rvw))*60) #Gear ratio for top speed

print("Engine Torque (max) needed :", Tmax_axle/GearRatio)

#Run taxi simulation
#--------------------PART 2---------------------------------------------------
max_a = DesAccel     #Maximum acceleration

#Taxiway from D14 to runway 36C
#taxiway = np.array([[21.33,35.52,31.68,43.17,105.66,60.91,1383,120,950,80,60],
#                    [0,38.8,0,44.8,0,49.5,0,105,0,43.6,52.6]])

#Taxiway from D14 to Polderbaan
taxiway = np.array([[21.33,35.52,31.68,43.17,105.66,60.91,1383,120,754,140,893,70,210,40,130,160,2140,130,1690,150,360],
                    [0,38.8,0,44.8,0,49.5,0,105,0,75.8,0,90.6,0,94,0,101.5,0,172,0,107.5,0]])

#Taxiwayid show whether we have straight part (st) or corner (cr)

#Taxiway ID from D14 to runway 36C
#taxiwayid = np.array(['st','cr','st','cr','st','cr','st','cr','st','cr','cr'])

#Taxiway ID from D14 to Polderbaan
taxiwayid = np.array(['st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st','cr','st'])
#First row is distance of straight part or corner
#If corner, second row gives turn radius -> otherwise 0
#In this code, the second row in the array is never used. However might be useful to make it more accurate.

#Simulation parameters
t = 0               #Starting time
dt = 0.01           #Time step

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
        
        indstart = ind                                      #Starting index in while loop
        v = varray[indstart]                                #Starting velocity in straight part
            
        while s<(taxiway[0][i]+sarray[indstart]):
                 
            a = max_a                                       #Speeding up with max acceleration    
            v = v + a*dt 
            if v > max_v:
                v = max_v
                a = 0                                       #If maximum speed is achieved, it does not need to accelerate anymore
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
            
            indnew = ind - int(round(t_braking/dt))         #Go back in time-> this is the index where we will start braking

            if indnew<0:                                    #Added after performing verification for high accelerations
                indnew=0

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
            

print("Delta taxi time [s] to Polderbaan: ",(tarray[-1])-std_taxtime )
#-----------------------------------------------------------------------------
time = tarray
acceleration = aarray
        
            
#Set up arrays
SectionTime = np.array([])
SectionAcceleration = np.array([])
SectionResForce = np.array([])
SectionCntrlForce = np.array([])
BeginTime = 0
LastAcceleration = 0
Energy = 0 

#Calculate sections of different acceleration
i = 1
while i < len(acceleration):
    if acceleration[i] != acceleration[i-1]:
        SectionTime = np.append(SectionTime,time[i]-BeginTime)
        SectionAcceleration = np.append(SectionAcceleration,acceleration[i-1])
        BeginTime = time[i]
    LastAcceleration = acceleration[i]
    i = i + 1

SectionTime = np.append(SectionTime,time[-1]-BeginTime)
SectionAcceleration = np.append(SectionAcceleration,LastAcceleration)

#Calculate input force for each section
for a in SectionAcceleration:
        SectionResForce = np.append(SectionResForce,MRW*a) 
        if a >= 0:
            SectionCntrlForce = np.append(SectionCntrlForce,((MRW*a)+DragRoll))
        else:
            SectionCntrlForce = np.append(SectionCntrlForce,((MRW*a)-DragRoll))

#Calculate energy for 1 car
i = 0
while i< len(SectionCntrlForce):
    if SectionCntrlForce[i]>ThrustSetMax-100:
        Energy = Energy + MaxPower*SectionTime[i]
    elif SectionCntrlForce[i]>DragRoll-100:
        Energy = Energy + (SectionCntrlForce[i]/ThrustSetMax)*MaxPower*SectionTime[i]
    i = i + 1

print("Energy needed for taxi (single vehicle): ", Energy/1000, "[mJ]")



        