#This program tries to find the max range for which total energy is still reduced

from math import exp, sqrt, pi
import numpy as np
import matplotlib.pyplot as plt

#Total mission:
#1-> Engine start and warm-up
#2-> Taxi
#3-> Take-off
#4-> Climb
#5-> Cruise
#6-> Loiter (reserve fuel)
#7 -> Descent
#8 -> Landing, taxi and shut-down

#------------------Input values-------------------------------------
V_cruise = 231.5                             #[m/s]
g = 9.81                                     #Gravitional constant [m/s^2]
SFC_cruise = 0.54                            #Specific fuel coefficient cruise [lb/lbf/hr]
SFC_loiter = 0.42                            #Specific fuel coefficient loiter [lb/lbf/hr]-> assumed from Roskam
k_cruise = 31.680                            #From ACFM data Paul for M=0.79 (cruise)-> higher because of Mach
CD0_cruise = 0.0231                          #Parasite drag coefficient from AFCM data Paul for M=0.79 (cruise)[-]
k_loiter = 25.939                            #From ACFM data Paul for M=0.342 (loiter)-> higher because of Mach
CD0_loiter = 0.0196                          #Parasite drag coefficient from AFCM data Paul for M=0.342 (loiter)[-]
E = 50*60                                    #Endurance of 50 min loiter at holding speed at 1500 ft in [s]

#Weights
MTOW = 97000                                 #Maximum take off weight [kg]
#MTOW = 93500                                #Maximum take off weight for validation [kg]
OEW_or = 47000*g                             #OEW A321neo [N]
W_PL = 28600*g                               #Payload weight [N]
#W_PL = 26390*g                              #Payload weight for validation [N]

#Fuel fractions from Roskam page 12
#W1_over_Wto is calculated as constant fuel added latter on in the code
#W2_over_W1 is calculated as constant fuel added latter on in the code
W3_over_W2 = 0.995                  #3-> Take-off
W4_over_W3 = 0.980                  #4-> Climb
#Determine W5_over_W4               #Fuel fraction cruise
#Determine W6_over_W5               #Fuel fraction loiter
W7_over_W6 = 0.990                  #7 ->Descent
#W8_over_W7 is calculated as constant fuel added latter on in the code

#LD_cruise calculation -> ADSEE formula for maximazing range
#Validation with Payload-Range diagram A321 manual
LD_cruise = 0.75 * sqrt((k_cruise)/(3*CD0_cruise))
print('Is value LD_cruise, ',LD_cruise, ', within range 13-15?') #Check if within common 13-15 (verification)

#LD_loiter calculation -> ADSEE formula for aerodynamic efficiency for maximum loiter time
CL_loiter = sqrt(k_loiter*CD0_loiter)
CD_loiter = 2*CD0_loiter
LD_loiter = CL_loiter/CD_loiter
print('Is value LD_loiter, ',LD_loiter, ', within range 14-18?') #Check if within common 14-18 (verification)

#Unit conversion
SFC_cruise = SFC_cruise * 0.283 * 10**-4     #Specific fuel coefficient cruise [lb/lbf/hr-> kg/Ns]
SFC_loiter = SFC_loiter * 0.283 * 10**-4     #Specific fuel coefficient loiter [lb/lbf/hr-> kg/Ns]

#Pushback Operations Fuel
Fuel_Cons_Truck = 0.231 * 175                                       #Fuel consumption conventional pushback truck per hour diesel [L/h]
Pushback_Time = 6 * 60                                              #Conventional pushback time [s]
Vfuel_pushback = (Pushback_Time/3600)*Fuel_Cons_Truck               #Fuel consumption conventional pushback truck diesel [L]
Cal_Val_Diesel = 36.9                                               #The calorific value of diesel fuel [MJ/litre]
Cal_Val_Diesel_2 = 45.5                                             #The calorific value of diesel fuel [MJ/kg]
Cal_Val_Jet = 43.1                                                  #The calorific value of Jet A-1 (kerosene) [MJ/kg]
Mfuel_pushback = (Vfuel_pushback*Cal_Val_Diesel)/Cal_Val_Jet        #Fuel used for pushback converted to Jet A-1 fuel [kg]

#Taxi Operations Fuel
Taxi_Outbound_Time = 13.475*60                                      #Conventional taxi time [s]
Fuel_Cons_Taxi = 0.24                                               #Fuel consumption engine based taxiing per second [kg/s]
Mfuel_taxi_outbound = Taxi_Outbound_Time * Fuel_Cons_Taxi           #Fuel consumption engine based outbound taxiing [kg]

Taxi_Inbound_Time = 9.475 * 60                                          #Average inbound taxi time excluding engine shut down time [s]
Mfuel_taxi_inbound = Taxi_Inbound_Time * Fuel_Cons_Taxi                 #Fuel consumption engine based inbound taxiing [kg]
Mfuel_landing = 125                                                     #Fuel consumption during landing is determined to be 125 [kg]
Mfuel_phase_8 = Mfuel_taxi_inbound + Mfuel_landing                      #Fuel for shut-down neglected
ECDT = 1.5*60                                                           #Engine cool down time with electrical system [s]
Fraction_Engine_On = (ECDT)/Taxi_Inbound_Time                           #If ZET-system is used, how long engine is still operating-> for now equal to engine shut down time

#Phase 1-> Engine start and warm-up
Warm_Up_Time_Engine = 4*60                                               #Engine warm up time without RETS system [s]
Warm_Up_Time_RETS = 2*60                                                 #Engine warm up time when RETS system is used [s]
Start_Up_Time = 2*60                                                     #Engine start up time of 2 min [s]
Mfuel_1_RETS = (Warm_Up_Time_RETS+Start_Up_Time)*Fuel_Cons_Taxi          #Fuel burned during phase 1 when RETS system is used [kg]
Mfuel_1_engine = (Warm_Up_Time_Engine+Start_Up_Time)*Fuel_Cons_Taxi      #Fuel burned during phase 1 without RETS system is used [kg]

'''
#---------------------------This part is introduced for validation--------------------------------- 

#Variables
Range = 1500                                 #Range [km]
addedweight = 0                              #Added weight by ZET-system [kg]

Range = Range*1000                           #Range [km-> m]

#New OEW
OEW = OEW_or + addedweight*9.81              #New OEW with ZET system included

#Brequet Range equation used for cruise
W5_over_W4 = 1/exp((Range*g*SFC_cruise)/(V_cruise*LD_cruise))

#Brequet endurance equation for loiter
W6_over_W5 = 1/exp((E*g*SFC_loiter)/(LD_loiter))

Mff = W3_over_W2*W4_over_W3*W5_over_W4*W6_over_W5*W7_over_W6
#Mused = 1-Mff

WTO = (OEW + W_PL + Mfuel_landing*g)/Mff                                                           #WTO not including fuel mass for phase 1 & 2 in [N]
Fuel_Total = ((1-Mff)*WTO)/g                                                                       #Fuel taken along at moment of take off [kg]
Mfuel = ((1-Mff/W6_over_W5)*WTO)/g + Mfuel_1_engine + Mfuel_taxi_outbound + Mfuel_phase_8          #Fuel used when no loiter needed [kg]

if (WTO/g)>MTOW:                        #Check if Maximum Take-off weight is exceeded
    print('Warning: Take off weight, ', WTO / g, ' kg, exceed MTOW of ',MTOW,'[kg]')

#Validation
print('The outbound and inbound taxi phase takes into account',(Mfuel_taxi_outbound+Mfuel_phase_8-Mfuel_landing)/Mfuel, 'of total fuel consumption')

'''

#Making Plots
n = 5                                                               #N+1 different weight between 0 and 1000, equally spaced
addedweightlst = np.append(np.array([0]),np.linspace(0,700,n))
#addedweightlst = np.array([   0.        ,  0,  222.22222222,  333.33333333, 444.44444444,  555.55555556,  666.66666667,  777.77777778,888.88888889, 1000.        ])
Rangelst = np.array([1806,2238.49,2816, 3366.29, 3856.31])          #Max range to cover 70,80,90,95,98% of all A321 flights
Mfuellst = np.zeros((len(Rangelst),n))                              #Matrix with columns: total fuel consumption in [kg] for given added weight; row gives range
WTOlst = np.zeros((len(Rangelst),n))                                #List of take-off weight in [kg] for given added weight

for i in range(len(Rangelst)):

    Range = Rangelst[i]

    # Unit conversion
    Range = Range * 1000                            # Range [km-> m]

    # Brequet Range equation used for cruise
    W5_over_W4 = 1 / exp((Range * g * SFC_cruise) / (V_cruise * LD_cruise))

    # Brequet endurance equation for loiter
    W6_over_W5 = 1 / exp((E * g * SFC_loiter) / (LD_loiter))

    for j in range(n):

        # New OEW
        OEW = OEW_or + addedweightlst[j]*9.81  # New OEW with ZET system included in [N]

        #if addedweightlst[j]!=0:               #In the case the RETS-system is used
        if j != 0:  # In the case the ZET-system is used
            Mff =  W3_over_W2 * W4_over_W3 * W5_over_W4 * W6_over_W5 * W7_over_W6
            WTO = (OEW + W_PL + Mfuel_landing*g)/Mff                                                                        # WTO in [N]
            Mfuel = ((1-Mff/W6_over_W5)*WTO)/g + Mfuel_1_RETS + Mfuel_landing + Fraction_Engine_On*Mfuel_taxi_inbound       # Fuel used (assumes no loiter) [kg]

        else:                                  #Baseline if ZET-system is not used and thus engine based!
            Mff = W3_over_W2 * W4_over_W3 * W5_over_W4 * W6_over_W5 * W7_over_W6
            print('For Range',Range, '[m] Mff is equal to', Mff)
            Mused = 1 - Mff

            WTO = (OEW + W_PL + Mfuel_landing*g)/Mff                                                                         # WTO in [N]
            Mfuel = ((1-Mff/W6_over_W5)*WTO)/g + Mfuel_1_engine + Mfuel_taxi_outbound + Mfuel_phase_8                        # Fuel used (assumes no loiter) [kg]

        Mfuellst[i,j]=Mfuel                       #Appending to list-> in kg
        WTOlst[i,j]=WTO/g                         #Appending to list-> in kg

        if (WTO / g) > MTOW:  # Check if Maximum Take-off weight is exceeded
            print('Warning: Take off weight, ', WTO / g, ' kg, exceed MTOW of 97000 [kg] for added weight of', addedweightlst[j], '[kg] and range of', Rangelst[i], '[km]')

f, ax = plt.subplots(1)
plt.grid()
ax.plot(addedweightlst[1:n],Mfuellst[0,0]-Mfuellst[0,1:n], addedweightlst[1:n],Mfuellst[1,0]-Mfuellst[1,1:n], addedweightlst[1:n], Mfuellst[2,0]-Mfuellst[2,1:n], addedweightlst[1:n], Mfuellst[3,0]-Mfuellst[3,1:n],addedweightlst[1:n], Mfuellst[4,0]-Mfuellst[4,1:n])
#plt.plot(addedweightlst[1:n],Mfuellst[0,1:n]-Mfuellst[0,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[1,1:n]-Mfuellst[1,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[2,1:n]-Mfuellst[2,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[3,1:n]-Mfuellst[3,0]-Mfuel_pushback,addedweightlst[1:n], Mfuellst[4,1:n]-Mfuellst[4,0]-Mfuel_pushback)
ax.set_xlim(left=0)
ax.legend(['Range = 1806 [km]; 70% A321 flights', 'Range = 2239 [km]; 80% A321 flights', 'Range = 2816 [km]; 90% A321 flights', 'Range = 3366 [km]; 95% A321 flights','Range = 3856 [km]; 98% A321 flights'], fontsize=18)
plt.xlabel('Added weight to the aircraft [kg]',fontsize=18)
plt.ylabel('Onboard fuel savings for certain range [kg]',fontsize=16)
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
#plt.savefig('Onboard_Fuel_Saving.PDF')
plt.show(f)

Fuel_Cons_APU = 0.0275                                                                            #Fuel consumption APU for providing total 100 [kW] (pneumatic+electric) [kg/s]
Mfuel_APU = Taxi_Outbound_Time*Fuel_Cons_APU + (Taxi_Inbound_Time-ECDT)*Fuel_Cons_APU             #Extra fuel burned APU [kg]
Mdiesel_RETS = 6.851946377                                                                        #Fuel for RETS in diesel coming from Power Department [kg] (excluding electricity batteries)
Elec_RETS = 94.17210464                                                                           #Amount of electricity needed by RETS coming from Power Department [MJ]
Mfuel_RETS = Mdiesel_RETS*(Cal_Val_Diesel_2/Cal_Val_Jet) + Elec_RETS/Cal_Val_Jet                  #Fuel for RETS in diesel [kg]
#Cal_Val_Diesel_2 =

Mfuel_additional = -Mfuel_pushback+Mfuel_APU+Mfuel_RETS

g, bx = plt.subplots(1)
plt.grid()
bx.plot(addedweightlst[1:n],Mfuellst[0,0]-Mfuellst[0,1:n]-Mfuel_additional, addedweightlst[1:n],Mfuellst[1,0]-Mfuellst[1,1:n]-Mfuel_additional, addedweightlst[1:n], Mfuellst[2,0]-Mfuellst[2,1:n]-Mfuel_additional, addedweightlst[1:n], Mfuellst[3,0]-Mfuellst[3,1:n]-Mfuel_additional,addedweightlst[1:n], Mfuellst[4,0]-Mfuellst[4,1:n]-Mfuel_additional)
#plt.plot(addedweightlst[1:n],Mfuellst[0,1:n]-Mfuellst[0,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[1,1:n]-Mfuellst[1,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[2,1:n]-Mfuellst[2,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[3,1:n]-Mfuellst[3,0]-Mfuel_pushback,addedweightlst[1:n], Mfuellst[4,1:n]-Mfuellst[4,0]-Mfuel_pushback)
bx.set_xlim(left=0)
bx.legend(['Range = 1806 [km]; 70% A321 flights', 'Range = 2239 [km]; 80% A321 flights', 'Range = 2816 [km]; 90% A321 flights', 'Range = 3366 [km]; 95% A321 flights','Range = 3856 [km]; 98% A321 flights'], fontsize=18)
plt.xlabel('Added weight to the aircraft [kg]',fontsize=18)
plt.ylabel('Final fuel savings for certain range [kg]',fontsize=16)
plt.xticks(fontsize=16)
plt.yticks(fontsize=16)
#plt.savefig('Onboard_Fuel_Saving.PDF')
plt.show(g)

