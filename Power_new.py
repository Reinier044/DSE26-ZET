import matplotlib.pyplot as plt

# ----------------------------------------------------- Functions ------------------------------------------------------


def stat_traction(torque, N, wheelrad, fric=1):

    if torque > (fric*N)*wheelrad:
        #print("Too much torque, will slip")
        return False
    return True


def EGTS_power(a, v, ratio, pow_wheel=2):

    w_rad_air       = 1.27      # [m] wheel radius aircraft MLG wheels
    m_plane         = 97400     # [kg] MRW
    m_car           = 20000     # [kg] Weight of external vehicle
    m_tot = m_plane + m_car
    weight_ratio    = 0.952     # [-] Weight distribution ratio

    F_tot = m_tot*a  # [N] Total force req to move plane at acceleration
    F_mlg = (1-ratio)*F_tot  # [N] Force needed  from internal
    F_mlg_w = F_mlg/pow_wheel  # [N] Force needed  from internal per wheel

    T_mlg_w = F_mlg_w*w_rad_air

    N_mlg   =  m_plane*weight_ratio*9.81
    N_mlg_w =  N_mlg/4
    if stat_traction(T_mlg_w, N_mlg_w, w_rad_air):
        print("Static friction checked")
    else:
        raise ValueError("Exceeds Static friction")

    w = v/w_rad_air
    return T_mlg_w*w

def car_power(a, v, ratio, pow_wheel=4):

    w_rad_car_1      = 0.537     # [m] wheel radius front tires external truck
    w_rad_car_2      = 0.496     # [m] wheel radius rear tires external truck
    m_plane         = 97400     # [kg] MRW
    m_car           = 20000     # [kg] Weight of external vehicle
    m_tot = m_plane + m_car
    weight_ratio    = 0.952     # [-] Weight distribution ratio

    F_tot = m_tot*a  # [N] Total force req to move plane at acceleration
    F_nlg = ratio*F_tot  # [N] Force needed  from internal
    F_nlg_w = F_nlg/pow_wheel  # [N] Force needed  from internal per wheel

    T_nlg_w_1 = F_nlg_w*w_rad_car_1
    T_nlg_w_2 = F_nlg_w*w_rad_car_2


    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81
    N_nlg_w = N_nlg/4
    if stat_traction(T_nlg_w_1, N_nlg_w, w_rad_car_1):
        print("Static friction checked rear wheels.")
    else:
        raise ValueError("Exceeds Static friction")
    if stat_traction(T_nlg_w_2, N_nlg_w, w_rad_car_2):
            print("Static friction checked front wheels.")
    else:
        raise ValueError("Exceeds Static friction")

    w_1 = v/w_rad_car_1
    w_2 = v/w_rad_car_2
    return T_nlg_w_1*w_1, T_nlg_w_2*w_2

def s_v_a_plotter(time, displacement, velocity, acceleration, layout='v'):
    """
    :param layout: layout of plots
        'v': vertical
        'h': horizontal
    :param time: time array
    :param displacement: displacement array
    :param velocity: velocity array
    :param acceleration: acceleration array
    :return: nothing
    """

    if layout == 'h':
        n = 130
    else:
        n = 310

    plt.figure(figsize=(20,10))

    plt.subplot(n+1)
    plt.title("Displacement")
    plt.xlabel("Time [s]")
    plt.ylabel("Displacement [m]")
    plt.plot(time, displacement)

    plt.subplot(n+2)
    plt.title("Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.plot(time, velocity)

    plt.subplot(n+3)
    plt.title("Acceleration")
    plt.xlabel("Time [s]")
    plt.ylabel("Acceleration [m/s/s]")
    plt.plot(time, acceleration)

    plt.tight_layout()
    plt.show()
    pass