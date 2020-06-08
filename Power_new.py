import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
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
    Roll_fric       = 0.02      # [-] Rolling friction coefficient

    N_mlg   =  m_plane*weight_ratio*9.81
    N_mlg_w =  N_mlg/4

    F_tot = m_tot*a + Roll_fric*m_tot*9.81  # [N] Total force req to move plane at acceleration
    F_mlg = (1-ratio)*F_tot  # [N] Force needed  from internal
    F_mlg_w = F_mlg/pow_wheel  # [N] Force needed  from internal per wheel

    T_mlg_w = F_mlg_w*w_rad_air

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
    Roll_fric       = 0.02      # [-] Rolling friction coefficient

    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81
    N_nlg_w = N_nlg/4

    F_tot = m_tot*a + Roll_fric*m_tot*9.81  # [N] Total force req to move plane at acceleration
    F_nlg = ratio*F_tot  # [N] Force needed  from internal
    print(F_nlg)
    F_nlg_w = F_nlg/pow_wheel  # [N] Force needed  from internal per wheel

    T_nlg_w_1 = F_nlg_w*w_rad_car_1
    T_nlg_w_2 = F_nlg_w*w_rad_car_2


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
    return T_nlg_w_1*w_1,T_nlg_w_2*w_2


def s_v_a_plotter(title,time, power, velocity, acceleration, layout='v'):
    """
    :param title: title of plots
        'egts': On aircraft power
        'car': Car power
        ' ': No title
    :param layout: layout of plots
        'v': vertical
        'h': horizontal
    :param time: time array
    :param power: power array
    :param velocity: velocity array
    :param acceleration: acceleration array
    :return: nothing
    """

    if layout == 'h':
        n = 130
    else:
        n = 310

    gs = gridspec.GridSpec(2, 2)

    fig = plt.figure()
    if title is 'egts':
        fig.suptitle("On Aircraft Power")
        ax1 = fig.add_subplot(gs[1, :])
        ax1.set_title("Power")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Power [kW]")
        ax1.plot(time, [i/1000 for i in power])
        ax0.plot(time, [52 for i in time], color='grey', linestyle='--')
    elif title is 'car':
        fig.suptitle("Vehicle Power")
        ax0 = fig.add_subplot(gs[1, 0])
        ax0.set_title("Power Front Wheel")
        ax0.set_xlabel("Time [s]")
        ax0.set_ylabel("Power [kW]")
        ax0.plot(time, [i/1000 for i in power[0, :]])
        ax1 = fig.add_subplot(gs[1, 1])
        ax1.set_title("Power Rear Wheel")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Power [kW]")
        ax1.plot(time, [i/1000 for i in power[1, :]])


    ax2 = fig.add_subplot(gs[0, 0])
    ax2.set_title("Velocity")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.plot(time, velocity, color='g')

    ax3 = fig.add_subplot(gs[0, 1])
    ax3.set_title("Acceleration")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Acceleration [$m/s^2$]")
    ax3.set_ylim(0, max(acceleration)+0.2)
    ax3.plot(time, acceleration, color='r')

    plt.tight_layout()
    plt.show()
    pass

def v_t(a,t):
    v = [0]
    for i in range(1,len(a)):
        v.append(a[i]*(t[i]-t[i-1]) + v[i-1])
    return v

power_ratio = 0.7
a = [1, 1, 1, 1, 1, 1, 1, 1, 1, 0.75, 0.75, 0.75, 0.75, 0.6, 0.6, 0.5, 0.4, 0.4, 0.3, 0.3, 0.3, 0.2, 0.2, 0.2]
t = [i for i in range((len(a)))]
v = v_t(a, t)

P_egts_w = []
for i in range(len(a)):
    P_egts_w.append(EGTS_power(a[i], v[i], power_ratio))

P_car_w = np.zeros((2,len(a)))
for i in range(len(a)):
    P_car_w[0,i], P_car_w[1,i] = car_power(a[i], v[i], power_ratio)

s_v_a_plotter('egts',t, P_egts_w, v, a)
s_v_a_plotter('car',t, P_car_w, v, a)
