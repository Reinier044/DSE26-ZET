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
#7 ->Descent
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
#MTOW = 97000                                 #Maximum take off weight [kg]
MTOW = 93500                                #Maximum take off weight for validation [kg]
OEW_or = 47000*g                             #OEW A321neo [N]
#W_PL = 28600*g                               #Payload weight [N]
W_PL = 26389*g                              #Payload weight for validation [N]

#Fuel fractions from Roskam page 12
W1_over_WTO = 0.990
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
Cal_Val_Diesel = 38.6                                               #The calorific value of diesel fuel [MJ/litre]
Cal_Val_Jet = 43.15                                                 #The calorific value of Jet A-1 (kerosene) [MJ/kg]
Mfuel_pushback = (Vfuel_pushback*Cal_Val_Diesel)/Cal_Val_Jet        #Fuel used for pushback converted to Jet A-1 fuel [kg]

#Taxi Operations Fuel
Taxi_Outbound_Time = 13.475*60                                      #Conventional taxi time [s]
Fuel_Cons_Taxi = 0.24                                               #Fuel consumption engine based taxiing per second [kg/s]
Mfuel_taxi_outbound = Taxi_Outbound_Time * Fuel_Cons_Taxi           #Fuel consumption engine based outbound taxiing [kg]

Taxi_Inbound_Time = 9.475 * 60                                          #Average inbound taxi time excluding engine shut down time [s]
Mfuel_taxi_inbound = Taxi_Inbound_Time * Fuel_Cons_Taxi                 #Fuel consumption engine based inbound taxiing [kg]
Mfuel_phase_8 = Mfuel_taxi_inbound + 125                                #Fuel for shut-down neglected, fuel consumption during landing is determined to be 125 [kg]
ECDT = 1.5*60                                                           #Engine cool down time with electrical system [s]
Fraction_Engine_On = (ECDT)/Taxi_Inbound_Time                           #If ZET-system is used, how long engine is still operating-> for now equal to engine shut down time

#Phase 1-> Engine start and warm-up
Warm_Up_Time_Engine = 4*60                                               #Engine warm up time without RETS system [s]
Warm_Up_Time_RETS = 2*60                                                 #Engine warm up time when RETS system is used [s]
Start_Up_Time = 2*60                                                     #Engine start up time of 2 min [s]
Mfuel_1_RETS = (Warm_Up_Time_RETS+Start_Up_Time)*Fuel_Cons_Taxi          #Fuel burned during phase 1 when RETS system is used [kg]
Mfuel_1_engine = (Warm_Up_Time_Engine+Start_Up_Time)*Fuel_Cons_Taxi      #Fuel burned during phase 1 without RETS system is used [kg]


#Variables
Range = 4716.9                               #Range [km]
addedweight = 0                              #Added weight by ZET-system [kg]

Range = Range*1000                           #Range [km-> m]

#New OEW
OEW = OEW_or + addedweight*9.81              #New OEW with ZET system included

#Brequet Range equation used for cruise
W5_over_W4 = 1/exp((Range*g*SFC_cruise)/(V_cruise*LD_cruise))

#Brequet endurance equation for loiter
#W6_over_W5 = 1/exp((E*g*SFC_loiter)/(LD_loiter))
W6_over_W5 = 1                                                   #If reserve fuel is not accounted for

Mff = W1_over_WTO*W3_over_W2*W4_over_W3*W5_over_W4*W6_over_W5*W7_over_W6
#Mused = 1-Mff

WTO = (OEW + W_PL)/Mff                                                                  #WTO not including fuel mass for phase 1 & 2 in [N]
#Mfuel = (Mused*WTO)/g + Mfuel_1_engine + Mfuel_taxi_outbound + Mfuel_phase_8           #Fuel used in [kg]

Mfuel = ((1-Mff/W6_over_W5)*WTO)/g + Mfuel_taxi_outbound + Mfuel_phase_8          #Fuel used in when no loiter needed [kg]

if (WTO/g)>MTOW:                        #Check if Maximum Take-off weight is exceeded
    print('Warning: Take off weight, ', WTO / g, ' kg, exceed MTOW of ',MTOW,'[kg]')