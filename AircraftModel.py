# -*- coding: utf-8 -*-
"""
Created on Wed May 13 10:14:36 2020
Created on Wed May 13 14:53:58 2020

@author: Reinier
@author: stijn
"""
import numpy as np

#Size data
NGx     = 5.07      #[m] Position of NG on x-axis
MLGx    = 21.97     #[m] Position of MLG on x-axis
MACcg   = 0.3607    #[%] cg position
XLEMAC  = 19.611397    #[m] 
MAC     = 4.29      #[m]
Hfslg   = 6.01      #[m] Fuselage height
Dfslg   = 4.14      #[m] Fuselage diameter
Rmlg    = 0.635     #[m] diameter MLG
MRW     = 97400     #[kg] Maximum ramp weight
g       = 9.81      #[m/s^2] Great 
Rvw     = 0.25      #[m] Radius Vehicle Wheel


#Variables
TaxiSpd         = 12.86 #[m/s]
Slope           = 0.0 #[rad] 0.0300196631 max (aka 3%)
CGfslg          = 1/3 #[%] assumed position of cg wrt fuselage (0 is bottom, 1 is top)
LiftNG          = 0.05 #[m] How high the nosegear is lifted
LiftMLG         = 0.05 #[m] How high the main landing gear is lifted
MuRolStatDry    = 0.02 #[-]Static friction coefficient dry surface
MuRolDynDry     = 0.02 #[-]Dynamic friction coefficient dry surface
MuKinDry        = 0.8 #[-]range of 0.6 - 0.85
MuKinWet        = 0.5 #[-]range of 0.45 - 0.75
GearRatio       = 14.95 #Gear ratio
ThrustSetMax    = 90000 #[N]Thrust setting
max_d           = -1.5 #Maximum deceleration -> should be negative valu5.144e!
max_v           = 12.86 #Maximum achievable velocity -> 30 kts
v_cr            = 5.144 #Limit on speed on turns (approx 10 kts)

#Calculate cg position
Liftslope   = np.arctan((LiftMLG-LiftNG)/(MLGx-NGx))
cgx         = MACcg*MAC+XLEMAC
cgy         = CGfslg*Dfslg+Hfslg-Dfslg+(LiftNG+(np.tan(Liftslope)*(cgx-NGx)))
cgz         = 0

#Forces for limit thrust
Weightx         = MRW*g*np.sin(Slope)
Weighty         = MRW*g*np.cos(Slope)
ThrustArm       = cgy - Rmlg #[m]
MLGnormalstat   = (Weightx*(ThrustArm)+(Weighty*(cgx-NGx)))/((cgx-NGx)+(MLGx-cgx))
NGnormalstat    = Weighty-MLGnormalstat
NGnormal        = 0 #[N] Main landing gear normal force
MLGnormal       = MRW*g*np.cos(Slope) - NGnormal #[N] Main landing gear normal force
MaxThrust       = (MLGnormal*(MLGx-cgx) - NGnormal*(cgx-NGx))/ThrustArm #[N]Maximum Force before tipover (+ DragNG*cgy + DragMLG*cgy)

#Calculate normal Forces
NGnormal    = NGnormalstat-(NGnormalstat/391000.95031004713)*ThrustSetMax
MLGnormal   = (MRW*np.cos(Slope)*g)-NGnormal

#Calculate drag
FrolStat    = MuRolStatDry*MLGnormal #[N] for static friction
DragNG      = MuRolDynDry*NGnormal #[N] for dynamic friction MLG
DragMLG     = MuRolDynDry*MLGnormal #[N] for dynamic friction NG
DragRoll    = DragNG + DragMLG
ResThrust   = ThrustSetMax-Weightx-DragRoll

#Calculate min and max force and torques
Fperwheel = ThrustSetMax/4
MinTorque = (FrolStat/6) *Rvw
Torque = ((ThrustSetMax+DragRoll-Weightx)/6) *Rvw
Tmax_axle = 2*Torque
Tmin_axle = 2*MinTorque

print("Time to top speed [s]:",max_v/(ResThrust/MRW))
print("Max accelaration [m/s^2]", ResThrust/MRW )
print("Engine Torque needed @MaxAcceleration:", Tmax_axle/GearRatio)



#Run taxi simulation
#--------------------PART 2---------------------------------------------------
max_a = ResThrust/MRW     #Maximum acceleration

taxiway = np.array([[21.33,35.52,31.68,43.17,105.66,60.91,1383,120,950,80,60],
                    [0,38.8,0,44.8,0,49.5,0,105,0,43.6,52.6]])
#First row is distance of straight part or corner
#If corner, second row gives turn radius -> otherwise 0

taxiwayid = np.array(['st','cr','st','cr','st','cr','st','cr','st','cr','cr'])
#Taxiwayid show whether we have straight part (st) or corner (cr)


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

#Calculate resultant force for each section
for a in SectionAcceleration:
        SectionResForce = np.append(SectionResForce,MRW*a) 
        if a >= 0:
            SectionCntrlForce = np.append(SectionCntrlForce,((MRW*a)+DragRoll))
        else:
            SectionCntrlForce = np.append(SectionCntrlForce,((MRW*a)-DragRoll))
