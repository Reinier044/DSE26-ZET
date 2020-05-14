# -*- coding: utf-8 -*-
"""
Created on Wed May 13 10:14:36 2020
Created on Wed May 13 14:53:58 2020

@author: Reinier
@author: stijn
"""
import numpy as np
import operations as ops

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
TaxiSpd     = 12.86 #[m/s]
Slope       = 0.0 #[rad] 0.0300196631 max (aka 3%)
CGfslg      = 1/3 #[%] assumed position of cg wrt fuselage (0 is bottom, 1 is top)
LiftNG      = 0.05 #[m] How high the nosegear is lifted
LiftMLG     = 0.05 #[m] How high the main landing gear is lifted
Liftslope   = np.arctan((LiftMLG-LiftNG)/(MLGx-NGx))
MuRolDry    = 0.02
MuKinDry    = 0.8 #[-]range of 0.6 - 0.85
MuKinWet    = 0.5 #[-]range of 0.45 - 0.75
GearRatio   = 14.95 #

#Calculate cg position
cgx = MACcg*MAC+XLEMAC
cgy = CGfslg*Dfslg+Hfslg-Dfslg+(LiftNG+(np.tan(Liftslope)*(cgx-NGx)))
cgz = 0


#Forces for limit thrust
Weightx         = MRW*g*np.sin(Slope)
Weighty         = MRW*g*np.cos(Slope)
ThrustArm       = cgy - Rmlg #[m]
TonSlope        = Weightx
MLGnormalstat   = (TonSlope*(ThrustArm)+(Weighty*(cgx-NGx)))/((cgx-NGx)+(MLGx-cgx))
NGnormalstat    = Weighty-MLGnormalstat
NGnormal        = 0 #[N] Main landing gear normal force
MLGnormal       = MRW*g*np.cos(Slope) - NGnormal #[N] Main landing gear normal force
DragNG          = 0 #[N]
DragMLG         = 0 #[N]
MaxThrust       = (MLGnormal*(MLGx-cgx) + DragNG*cgy + DragMLG*cgy - NGnormal*(cgx-NGx))/ThrustArm #[N]Maximum Force before tipover


#Sum of forces and moments
Sfx         = MRW*g*np.sin(Slope)+ DragNG + DragMLG - MaxThrust
Sfy         = NGnormal + MLGnormal - MRW*g*np.cos(Slope)
Smcgz       = NGnormal*(cgx-NGx)+ MaxThrust*ThrustArm - MLGnormal*(MLGx-cgx) - DragNG*cgy - DragMLG*cgy

ThrustSet   = 100000
ResThrust   = ThrustSet-TonSlope
NGnormal    = NGnormalstat-(NGnormalstat/391000.95031004713)*ThrustSet
MLGnormal   = (MRW*np.cos(Slope)*g)-NGnormal

Fperwheel = ThrustSet/4
Mureq= Fperwheel/(MLGnormalstat/2)

Frol    = MuRolDry*MLGnormal
Ffric   = MuKinDry*MLGnormal

MinTorque = (Frol/6) *Rvw
Torque = (ThrustSet/6) *Rvw
Tmax_axle = 2*Torque
Tmin_axle = 2*Torque

print("Time to top speed [s]:",TaxiSpd/(ResThrust/MRW))
print("Max accelaration [m/s^2]", ResThrust/MRW )
print("Engine Torque needed @MaxAcceleration:", Tmax_axle/GearRatio)

time = ops.tarray
accelaration = ops.aarray
