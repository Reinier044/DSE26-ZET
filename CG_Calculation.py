# -*- coding: utf-8 -*-
"""
Created on Tue Jun 02 16:19:57 2020

@author: stijn
"""
#Standard Values
g = 9.81

#A321 Characteristics
XLEMAC          = 19.611397     #[m] X Position mean aerodynamic chord
MAC             = 4.29          #[m] Length mean aerodynamic chord
FWD_CG_old_mac  = 0.175          #[%] measured as % of MAC
AFT_CG_old_mac  = 0.3802         #[%] measured as % of MAC
OEW_CG_old_mac  = 0.25          #[%] measured as % of MAC
CG_W_add        = 21.97         #[m] measured from tip of airplane

m_A321 = 47000          #[kg]
W_A321 = m_A321*g       #[N]

m_add = 700             #[kg]
W_add = m_add*g         #[N]

FWD_CG_old = FWD_CG_old_mac * MAC + XLEMAC
AFT_CG_old = AFT_CG_old_mac * MAC + XLEMAC
OEW_CG_old = OEW_CG_old_mac * MAC + XLEMAC

#Calculation for new CG
FWD_CG_new = (W_add/W_A321) * (CG_W_add - FWD_CG_old) + FWD_CG_old
AFT_CG_new = (W_add/W_A321) * (CG_W_add - AFT_CG_old) + AFT_CG_old
OEW_CG_new = (W_add/W_A321) * (CG_W_add - OEW_CG_old) + OEW_CG_old

FWD_CG_new_mac = (FWD_CG_new - XLEMAC)/MAC
AFT_CG_new_mac = (AFT_CG_new - XLEMAC)/MAC
OEW_CG_new_mac = (OEW_CG_new - XLEMAC)/MAC

FWD_CG_delta = FWD_CG_new - FWD_CG_old
AFT_CG_delta = AFT_CG_new - AFT_CG_old
OEW_CG_delta = OEW_CG_new - OEW_CG_old

print("FWD_CG_delta = " , FWD_CG_delta)
print("FWD_CG_delta = " , FWD_CG_delta)
print("FWD_CG_delta = " , FWD_CG_delta)