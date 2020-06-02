"""
Created on TUES JUN 2 15:18:14 2020
@author: Willem
File to size MLG engines and calculate their performance
"""

# Inputs ------------------------------------------------------------------
# Power Electrical motor
EGTS_power_in = 10000/2  # [W] Electrical motor power in/ gear
motor_eff = 1  # [-] Electrical motor efficiency
EGTS_power_out = EGTS_power_in*motor_eff

# Weights
mrw = 97400  # [kg] Maximum ramp weight  !Note: Additional system weight not included!
mtow = 93500  # [kg] Maximum take-off weight  !Note: Additional system weight not included!
MLG_percent = 95.2  # [%] Percentage of weight on MLG
NLG_percent = 1 - MLG_percent  # [%] Percentage of weight on NLG
# Resistance
Roll_res = 0.02  # [-] Rolling resistance
Taxi_slope = 0.03  # [-]
Dyn_fric = 0.8  # [-] Dynamic friction coefficient (concrete)
Stat_fric = 1  # [-] Static friction coefficient (concrete)

