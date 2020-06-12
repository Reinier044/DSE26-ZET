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
e = 0.83                                     #Oswald efficiency factor-> assumed from literature-> check with Paul?
b = 35.8                                     #Wingspan [m]
S = 122.4                                    #Wingsurface [m^2]
A = (b**2)/S                                 #Aspect ratio [-] -> not effective Aspect ratio (+1.4) !!
#k = pi*A*e
k = 43.831                                   #From ACFM data Paul-> higher because of Mach
CD0 = 0.0272                                 #Parasite drag coefficient from AFCM data Paul [-]
E = 50*60                                    #Endurance of 50 min loiter at holding speed at 1500 ft in [s]

#Weights
MTOW = 97000                                 #Maximum take off weight [kg]
OEW_or = 47000*g                             #OEW A321neo [N]
#W_PL = 28600*g                              #Payload weight [N]
W_PL = 26389*g                               #Payload weight for validation [N]

#Fuel fractions from Roskam page 12
W1_over_Wto = 0.990
#W2_over_W1 is calculated as constant fuel added latter on in the code
W3_over_W2 = 0.995
W4_over_W3 = 0.980
#Determine W5_over_W4                        #Fuel fraction cruise
#Determine W6_over_W5                        #Fuel fraction loiter
W7_over_W6 = 0.990
#W8_over_W7 is calculated as constant fuel added latter on in the code

#LD_cruise calculation -> ADSEE formula for maximazing range
#Validation with Payload-Range diagram A321 manual
LD_cruise = 0.75 * sqrt((k)/(3*CD0))
print('Is value LD_cruise, ',LD_cruise, ', within range 13-15?') #Check if within common 13-15 (verification)

#LD_loiter calculation -> ADSEE formula for aerodynamic efficiency for maximum loiter time
CL_loiter = sqrt(k*CD0)
CD_loiter = 2*CD0
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
Fuel_Cons_Taxi = 2.25                                               #Fuel consumption engine based taxiing per second [kg/s]
Mfuel_taxi_outbound = Taxi_Outbound_Time * Fuel_Cons_Taxi           #Fuel consumption engine based outbound taxiing [kg]

Taxi_Inbound_Time = 9.475 * 60                                          #Average inbound taxi time excluding engine shut down time [s]
Fraction_Engine_On = (3*60)/Taxi_Inbound_Time                           #If ZET-system is used, how long engine is still operating-> for now equal to engine shut down time
Mfuel_taxi_inbound = Taxi_Inbound_Time * Fuel_Cons_Taxi                 #Fuel consumption engine based inbound taxiing [kg]

'''

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

Mff = W1_over_Wto*W3_over_W2*W4_over_W3*W5_over_W4*W6_over_W5*W7_over_W6
Mused = 1-Mff

WTO = (OEW + W_PL + Mfuel_taxi_inbound*9.81)/(1-Mused) + Mfuel_taxi_outbound*9.81                    #WTO in [N]
Mfuel = (Mused*(WTO-Mfuel_taxi_outbound*9.81))/g + Mfuel_taxi_outbound + Mfuel_taxi_inbound          #Fuel used in [kg]

if (WTO/g)>MTOW:                        #Check if Maximum Take-off weight is exceeded
    print('Warning: Take off weight, ', WTO / g, ' kg, exceed MTOW of 97000 [kg]')

'''

#Making Plots
n = 19                                              #N+1 different weight between 0 and 1000, equally spaced
addedweightlst = np.linspace(0,2000,n)
Rangelst = np.array([1538,1806,2239,2816])          #Max range to cover 60,70,80 and 90% of all A321 flights
Mfuellst = np.zeros((len(Rangelst),n))              #Matrix with columns: total fuel consumption in [kg] for given added weight; row gives range
WTOlst = np.zeros((len(Rangelst),n))                #List of take-off weight in [kg] for given added weight

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

        #if addedweightlst[j]!=0:               #In the case the ZET-system is used
        if j != 0:  # In the case the ZET-system is used
            Mff = W1_over_Wto * W3_over_W2 * W4_over_W3 * W5_over_W4 * W6_over_W5 * W7_over_W6

            Mused = 1 - Mff
            WTO = (OEW + W_PL + Fraction_Engine_On*Mfuel_taxi_inbound*g) / (1 - Mused)  # WTO in [N]
            Mfuel = (Mused * WTO) / g + Fraction_Engine_On*Mfuel_taxi_inbound               # Fuel used in [kg]

        else:                                  #Baseline if ZET-system is not used and thus engine based!
            Mff = W1_over_Wto * W3_over_W2 * W4_over_W3 * W5_over_W4 * W6_over_W5 * W7_over_W6
            print('For Range',Range, '[m] Mff is equal to', Mff)
            Mused = 1 - Mff
            WTO = (OEW + W_PL + Mfuel_taxi_inbound * g) / (1 - Mused) + Mfuel_taxi_outbound * g                 # WTO in [N]
            Mfuel = (Mused * (WTO - Mfuel_taxi_outbound * g)) / g + Mfuel_taxi_outbound + Mfuel_taxi_inbound    # Fuel used in [kg]

        Mfuellst[i,j]=Mfuel                       #Appending to list-> in kg
        WTOlst[i,j]=WTO/g                         #Appending to list-> in kg

        if (WTO / g) > MTOW:  # Check if Maximum Take-off weight is exceeded
            print('Warning: Take off weight, ', WTO / g, ' kg, exceed MTOW of 97000 [kg] for added weight of', addedweightlst[j], '[kg] and range of', Rangelst[i], '[km]')

plt.figure()
plt.grid()
plt.plot(addedweightlst[1:n],Mfuellst[0,1:n]-Mfuellst[0,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[1,1:n]-Mfuellst[1,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[2,1:n]-Mfuellst[2,0]-Mfuel_pushback, addedweightlst[1:n], Mfuellst[3,1:n]-Mfuellst[3,0]-Mfuel_pushback)
plt.legend(['Range = 1538 [km]; 60% A321 flights','Range = 1806 [km]; 70% A321 flights', 'Range = 2239 [km]; 80% A321 flights', 'Range = 2816 [km]; 90% A321 flights'])
plt.xlabel('Added weight in [kg]')
plt.ylabel('Extra fuel consumed in [kg]')

#.