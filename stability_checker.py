# -*- coding: utf-8 -*-
"""
Created on Tue Jun 02 16:19:57 2020
@author: Stijn & Mathieu
"""
import math as m
#=====Standard Values=====#
g = 9.81 #[m/s^2] Acceleration due to gravity


#=====A321 Characteristics=====#
XLEMAC          = 19.611397     #[m] X Position mean aerodynamic chord
MAC             = 4.29          #[m] Length mean aerodynamic chord
FWD_CG_old_mac  = 0.175         #[%] measured as % of MAC
AFT_CG_old_mac  = 0.3802        #[%] measured as % of MAC
OEW_CG_old_mac  = 0.25          #[%] measured as % of MAC
x_nlg           = 5.07          #[m] x-position of nose landing gear
x_mlg           = 21.97         #[m] x-position of main landing gear
theta           = 11            #[degree] <--CHECK
d_top           = 6.01          #[m] Fuselage height
d_fus           = 4.14          #[m] Fuselage diameter
m_A321          = 93900         #[kg] Maximum ramp mass
W_A321          = m_A321*g      #[N] maximum ramp weight


#=====Inputs=====#
m_add           = 700               #[kg] Mass of new components
W_add           = m_add*g           #[N] Weight of new components
CG_W_add        = 21.97             #[m] Center of gravity of new components
z_cg_old        = -2/3*d_fus+d_top  #[m] Old z-position of center of gravity
a_max           = 1                 #[m/s^2] Input new maximum acceleration at nose landing gear
z_cg_add        = 0.635             #[m] Input z-position of new mass


#=====New cg positions=====#
FWD_CG_old = FWD_CG_old_mac * MAC + XLEMAC
AFT_CG_old = AFT_CG_old_mac * MAC + XLEMAC
OEW_CG_old = OEW_CG_old_mac * MAC + XLEMAC

FWD_CG_new = (W_add/(W_A321+W_add)) * (CG_W_add - FWD_CG_old) + FWD_CG_old
AFT_CG_new = (W_add/(W_A321+W_add)) * (CG_W_add - AFT_CG_old) + AFT_CG_old
OEW_CG_new = (W_add/(W_A321+W_add)) * (CG_W_add - OEW_CG_old) + OEW_CG_old
z_cg_new   = (z_cg_old * W_A321 + z_cg_add * W_add) / (W_A321 + W_add)


FWD_CG_new_mac = (FWD_CG_new - XLEMAC)/MAC
AFT_CG_new_mac = (AFT_CG_new - XLEMAC)/MAC
OEW_CG_new_mac = (OEW_CG_new - XLEMAC)/MAC

FWD_CG_delta = FWD_CG_new - FWD_CG_old
AFT_CG_delta = AFT_CG_new - AFT_CG_old
OEW_CG_delta = OEW_CG_new - OEW_CG_old
z_CG_delta   = z_cg_new - z_cg_old

print("FWD_CG_delta = " , FWD_CG_delta)
print("AFT_CG_delta = " , AFT_CG_delta)
print("OEW_CG_delta = " , OEW_CG_delta)
print("z_CG_delta = " , z_CG_delta)

#=====Landing Tip-over check=====#
beta=m.degrees(m.atan((x_mlg-AFT_CG_new)/z_cg_new))
if beta>theta:
    print("Landing Tip-over ok")
else:
    print("Landing Tip-over not ok")

#=====Acceleration Tip-over check=====#
a_tip=g*(x_mlg-AFT_CG_new)/z_cg_new
if a_max<a_tip:
    print("Acceleration Tip-over ok")
else:
    print("Acceleration Tip-over not ok")

#=====Ground equilibrium=====#
nlg_weight_max=(x_mlg-FWD_CG_new)/(x_mlg-x_nlg)
nlg_weight_min=(x_mlg-AFT_CG_new)/(x_mlg-x_nlg)

if 0.15>nlg_weight_max and nlg_weight_min>0.08:
    print("Ground equilibrium ok")
else:
    if 0.15>nlg_weight_max:
        print("Nose gear traction insufficient")
    else:
        print("Nose gear weight too high")
