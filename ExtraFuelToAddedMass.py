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
k = 31.680                                   #From ACFM data Paul-> higher because of Mach
CD0 = 0.0231                                 #Parasite drag coefficient from AFCM data Paul [-]
E = 50*60                                    #Endurance of 50 min loiter at holding speed at 1500 ft in [s]

#Weights
#MTOW = 97000                                 #Maximum take off weight [kg]
MTOW = 93500                                  #Maximum take off weight for validation [kg]
OEW_or = 47000*g                              #OEW A321neo [N]
#W_PL = 28600*g                               #Payload weight [N]
W_PL = 26389*g                                #Payload weight for validation [N]

#Fuel fractions from Roskam page 12
W1_over_Wto = 0.990
W2_over_W1 = 0.990                           #This is for conventional taxiing-> change?
W3_over_W2 = 0.995
W4_over_W3 = 0.980
#Determine W5_over_W4                        #Fuel fraction cruise
#Determine W6_over_W5                        #Fuel fraction loiter
W7_over_W6 = 0.990
W8_over_W7 = 0.992                           #This is for conventional taxiing-> change?

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


#Variables
Range = 4716.9                                #Range [km]
addedweight = 0                              #Added weight by ZET-system [kg]

Range = Range*1000                           #Range [km-> m]

#New OEW
OEW = OEW_or + addedweight                   #New OEW with ZET system included

#Brequet Range equation used for cruise
W5_over_W4 = 1/exp((Range*g*SFC_cruise)/(V_cruise*LD_cruise))

#Brequet endurance equation for loiter
#W6_over_W5 = 1/exp((E*g*SFC_loiter)/(LD_loiter))
W6_over_W5 = 1                                                   #If reserve fuel is not accounted for

Mff = W1_over_Wto*W2_over_W1*W3_over_W2*W4_over_W3*W5_over_W4*W6_over_W5*W7_over_W6*W8_over_W7
Mused = 1-Mff

WTO = (OEW + W_PL)/(1-Mused)                #WTO in [N]
Mfuel = (Mused*WTO)/g                       #Fuel used in [kg]

if (WTO/g)>MTOW:                        #Check if Maximum Take-off weight is exceeded
    print('Warning: Take off weight, ', WTO / g, ' kg, exceed MTOW of ',MTOW,'[kg]')

'''
#Making Plots
n = 9                                               #N+1 different weight between 0 and 1000, equally spaced
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

    Mff = W1_over_Wto * W2_over_W1 * W3_over_W2 * W4_over_W3 * W5_over_W4 * W6_over_W5 * W7_over_W6 * W8_over_W7
    Mused = 1 - Mff

    for j in range(n):

        addedweightlst = np.linspace(0,1000,n)

        # New OEW
        OEW = OEW_or + addedweightlst[j]*9.81  # New OEW with ZET system included in [N]

        WTO = (OEW + W_PL) / (1 - Mused)        # WTO in [N]
        Mfuel = (Mused * WTO) / g               # Fuel used in [kg]

        Mfuellst[i,j]=Mfuel                       #Appending to list
        WTOlst[i,j]=WTO/g                         #Appending to list

        if (WTO / g) > MTOW:  # Check if Maximum Take-off weight is exceeded
            print('Warning: Take off weight, ', WTO / g, ' kg, exceed MTOW of 97000 [kg] for added weight of', addedweightlst[j], '[kg] and range of', Rangelst[i], '[km]')

plt.figure()
plt.plot(addedweightlst,Mfuellst[0,:]-Mfuellst[0,0], addedweightlst, Mfuellst[1,:]-Mfuellst[1,0], addedweightlst, Mfuellst[2,:]-Mfuellst[2,0], addedweightlst, Mfuellst[3,:]-Mfuellst[3,0])
plt.legend(['Range = 1538 [km]; 60% A321 flights','Range = 1806 [km]; 70% A321 flights', 'Range = 2239 [km]; 80% A321 flights', 'Range = 2816 [km]; 90% A321 flights'])
plt.xlabel('Added weight in [kg]')
plt.ylabel('Extra fuel consumed in [kg]')

'''