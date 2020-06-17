"""
The two function and corresponding equation. Note I did not  run that calculation exactly so the np.sum part might not work but you just need to sum the  returns of car power
"""

def EGTS_power(a, v, ratio: float, pow_wheel: int = 2):
    """
    Function that calculates the (peak) power required by each MLG engine based upon the acceleration and speed.
    (Removed stat fric check and torque return with respect to original in power calc)
    :param a: List of all the accelerations. [m/s^2]
    :param v: List of the corresponding velocities. [m/s]
    :param ratio: Ratio of (maximum) force that will be provided by the external vehicle. [-]
    :param pow_wheel: The number of powered MLG wheels. Automatically set to 2 which is the same as EGTS. [-]
    :return: Required power to put on each ring gear given the acceleration needed at a certain speed
    """
    # cte
    w_rad_air       = 1.27/2    # [m] wheel radius aircraft MLG wheels
    m_plane         = 97400     # [kg] MRW
    m_car           = 22000     # [kg] Mass of external vehicle
    m_tot = m_plane + m_car     # [kg] Total mass of the system
    weight_ratio    = 0.952     # [-] Landing gear weight distribution ratio
    Roll_fric       = 0.02      # [-] Rolling friction coefficient of airplane wheels
    Roll_fric_car   = 0.0065    # [-] Rolling friction coefficient of external vehicle wheels

    # Necessary force and Torque calculations
    N_mlg   = m_plane*weight_ratio*9.81                # [N] Total normal force on the MLG
    N_mlg_w = N_mlg/4                                  # [N] Normal force per MLG wheel
    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81  # [N] Total normal force of car

    F_tot = m_tot*a + Roll_fric*N_mlg + Roll_fric_car*N_nlg  # [N] Total force req to move plane at acceleration
    F_mlg = (1-ratio)*F_tot                                  # [N] Force needed  from internal
    F_mlg_w = F_mlg/pow_wheel                                # [N] Force needed  from internal per wheel

    T_mlg_w = F_mlg_w*w_rad_air  # [Nm] Torque for wheels (note on outer radius)

    # Rotational speed of wheels
    w = v/w_rad_air  # [rad/s] rotational speed wheel

    return T_mlg_w*w


def car_power(a, v, ratio, pow_wheel=4):
    """
    Function that calculates the (peak) power required to power each car wheel based upon the acceleration and speed.
    (Removed stat fric check with respect to original in power calc)
    :param a: List of all the accelerations. [m/s^2]
    :param v: List of the corresponding velocities. [m/s]
    :param ratio: Ratio of (maximum) force that will be provided by the external vehicle. [-]
    :param pow_wheel: The number of powered MLG wheels. Automatically set to 4 meaning the truck has to be 4WD [-].
    :return: Required power to put on each wheel given the acceleration needed at a certain speed.
    """
    # cte
    w_rad_car_1     = 0.537     # [m] wheel radius front tires external truck
    w_rad_car_2     = 0.537     # [m] wheel radius rear tires external truck 0.496
    m_plane         = 97400     # [kg] MRW
    m_car           = 22000     # [kg] Weight of external vehicle
    m_tot = m_plane + m_car     # [kg] Total mass of the system
    weight_ratio    = 0.952     # [-] Weight distribution ratio
    Roll_fric       = 0.02      # [-] Rolling friction coefficient for MLG gears
    Roll_fric_car   = 0.0065    # [-] Rolling friction coefficient for car wheels

    # Necessary force and Torque calculations
    N_mlg   =  m_plane*weight_ratio*9.81                    # [N] Total normal force on the MLG
    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81       # [N] Total normal force on the car
    N_nlg_w = N_nlg/4                                       # [N] Normal force per MLG wheel

    F_tot = m_tot*a + Roll_fric*N_mlg + Roll_fric_car*N_nlg  # [N] Total force req to move plane at acceleration
    F_nlg = ratio*F_tot                                      # [N] Force needed  from internal
    F_nlg_w = F_nlg/pow_wheel                                # [N] Force needed  from internal per wheel

    T_nlg_w_1 = F_nlg_w*w_rad_car_1            # [Nm] Torque per front wheel (note on outer radius)
    T_nlg_w_2 = F_nlg_w*w_rad_car_2            # [Nm] Torque per rear wheel (note on outer radius)

    # Rotational speed of wheels
    w_1 = v/w_rad_car_1  # [rad/s] rotational speed wheel
    w_2 = v/w_rad_car_2  # [rad/s] rotational speed wheel

    return T_nlg_w_1*w_1, T_nlg_w_2*w_2


P_tot = 2*EGTS_power(a,v,ratio)+ 2*np.sum([car_power((a,v,ratio))])