# -*- coding: utf-8 -*-
"""
Created on Wed May 13 10:14:36 2020

@author: Reinier
"""
import numpy as np

#Size data
NGx = 5.07 #[m] Position of NG on x-axis
MLGx = 21.97 #[m] Position of MLG on x-axis
MACcg = 0.3607 #[%] cg position
XLEMAC = 19.342 #[m] 
MAC = 4.26 #[m]
Hfslg = 6.01 #[m] Fuselage height
Dfslg = 4.14 #[m] Fuselage diameter
Rmlg = 0.635 #[m] diameter MLG
MRW = 97400 #[kg]
g = 9.80665


#Variables
TaxiSpd = 12.86 #[m/s]
Slope = 0.0300196631 #[rad] 0.0300196631 max (aka 3%)
CGfslg = 1/3 #[%] assumed position of cg wrt fuselage (0 is bottom, 1 is top)
LiftNG = 0.05 #[m] How high the nosegear is lifted
LiftMLG = 0.05 #[m] How high the main landing gear is lifted
Liftslope = np.arctan((LiftMLG-LiftNG)/(MLGx-NGx))

#Calculate cg position
cgx = MACcg*MAC+XLEMAC
cgy = CGfslg*Dfslg+Hfslg-Dfslg+(LiftNG+(np.tan(Liftslope)*(cgx-NGx)))
cgz = 0


#Forces for limit thrust
NGnormal = 0
MLGnormal = MRW*g*np.cos(Slope) - NGnormal
DragNG = 0
DragMLG = 0
ThrustArm = cgy - Rmlg #[m]
Thrust = (MLGnormal*(MLGx-cgx) + DragNG*cgy + DragMLG*cgy - NGnormal*(cgx-NGx))/ThrustArm


#Sum of forces and moments
Sfx = MRW*g*np.sin(Slope)+ DragNG + DragMLG - Thrust
Sfy = NGnormal + MLGnormal - MRW*g*np.cos(Slope)
Smcgz = NGnormal*(cgx-NGx)+ Thrust*ThrustArm - MLGnormal*(MLGx-cgx) - DragNG*cgy - DragMLG*cgy

print("Max Thrust [N]:", Thrust)
print("Max accelaration [m/s^2]:", Thrust/MRW)
print("Time to top speed [s]:",TaxiSpd/(Thrust/MRW))
