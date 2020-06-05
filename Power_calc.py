"""
Created on TUES JUN 2 14:53:58 2020

@author: Willem
"""
import numpy as np
import matplotlib.pyplot as plt
# ----------------------------------------------------- Functions ------------------------------------------------------


def enough_traction(torque, N, fric, u="stat", wheelrad=1.27):
    if u is "stat":
        if torque > (fric*N)*wheelrad:
            #print("Too much torque, will slip")
            return False
        return True
    elif u is "roll":
        if torque <= (fric*N)*wheelrad:
            #print("Not enough torque")
            return False
        return True
    return "unvalid input"


# ------------------------------------------------------- Inputs -------------------------------------------------------
# Power ratio external to internal
ratio           = 0.7       # [-]
pow_wheel_air   = 2
pow_wheel_car   = 4
# Acceleration and speed
at              = 1         # [m/s2] Acceleration with external
vt              = 30        # [knots] Top speed with external
# Masses
ratio_plane     = 0.952     # [-] % weight on mlg
m_plane         = 97400     # [kg] MRW
m_plane_add     = 0         # [kg] Added on aircraft weight
m_car           = 20000     # [kg] Weight of external vehicle

# -------------------------------------------------------- Data --------------------------------------------------------
# frictions
Roll_air        = 0.02
Stat_air_dry    = 1
#dimensions
w_rad_air       = 1.27      # [m] wheel radius aircraft MLG wheels
w_rad_car1      = 0.537     # [m] wheel radius front tires external truck
w_rad_car2      = 0.496     # [m] wheel radius rear tires external truck
# Conversion
ktstoms         = 0.514444  # [m/s/kts] 1 kts to m/s

# ---------------------------------------------------- Calculations ----------------------------------------------------
m_tot   = m_plane + m_plane_add + m_car  # [kg] Total mass
# Normal forces
N_nlg   = (m_car + m_plane*(1-ratio_plane))*9.81
N_mlg   = (m_plane_add + m_plane*ratio_plane)*9.81
N_mlg_w =  N_mlg/4

# Required towing forces
F_tot = m_tot*at  # [N] Total force req to move plane at acceleration
F_nlg = ratio*F_tot  # [N] Force needed  from external
F_mlg = (1-ratio)*F_tot  # [N] Force needed  from internal
F_mlg_w = F_mlg/pow_wheel_air  # [N] Force needed  from internal per wheel
"""if F_nlg > 23901.55:
    print("F_nlg = ",F_nlg)
    raise ValueError("This exceeds limit the NLG can  handle")"""

# Required torques
T_mlg_w = F_mlg_w*w_rad_air

T_nlg_w_1 = F_nlg/pow_wheel_car*w_rad_car1
T_nlg_w_2 = F_nlg/pow_wheel_car*w_rad_car2


# Min force
F_roll = Roll_air*m_tot*9.81
F_roll_nlg = ratio*F_roll
F_roll_mlg_w = (F_roll - F_roll_nlg)/pow_wheel_air
a_min = F_roll/m_tot

T_nlg_w_1 = F_nlg/pow_wheel_car*w_rad_car1
T_nlg_w_2 = F_nlg/pow_wheel_car*w_rad_car2



print("Total Force : ", F_tot)
print("External vehicle: Ftot_ex=", round(F_nlg, 2))
print("\t \t \t L \t \t \t R")
print("front \t", round(T_nlg_w_1, 2), " \t ", round(T_nlg_w_1, 2))
print("\t \t (", enough_traction(T_mlg_w, N_nlg/4, Stat_air_dry, "stat", w_rad_car1),
      ") \t (", enough_traction(T_mlg_w, N_nlg/4, Stat_air_dry, "stat", w_rad_car1), ")")
print("back \t", round(T_nlg_w_2, 2), " \t ", round(T_nlg_w_2, 2))
print("\t \t (", enough_traction(T_mlg_w, N_nlg/4, Stat_air_dry, "stat", w_rad_car2),
      ") \t (", enough_traction(T_mlg_w, N_nlg/4, Stat_air_dry, "stat", w_rad_car2), ")")
print("EGTS: Ftot_in=", round(F_mlg, 2))
print("\t \t \t L \t \t \t R")
print("Outer \t", round(T_mlg_w,  2), " \t ", round(T_mlg_w, 2))
print("\t \t (", enough_traction(T_mlg_w, N_mlg/4, Stat_air_dry, "stat"),
      ") \t (", enough_traction(T_mlg_w, N_mlg/4, Stat_air_dry, "stat"), ")")


def EGTS_power(a, v):
    w_rad_air       = 1.27      # [m] wheel radius aircraft MLG wheels
    F_tot = m_tot*a  # [N] Total force req to move plane at acceleration
    F_mlg = (1-ratio)*F_tot  # [N] Force needed  from internal
    F_mlg_w = F_mlg/pow_wheel_air  # [N] Force needed  from internal per wheel
    w