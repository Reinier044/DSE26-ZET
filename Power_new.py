import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
# ----------------------------------------------------- Functions ------------------------------------------------------


def opt_force_ratio(F_nlg_allow,a_lst):
    m_plane         = 97400     # [kg] MRW
    m_car           = 20000     # [kg] Weight of external vehicle
    m_tot = m_plane + m_car
    Roll_fric       = 0.02      # [-] Rolling friction coefficient

    F_tot = m_tot*max(a_lst) + Roll_fric*m_tot*9.81  # [N] Total force req to move plane at acceleration
    print("Max force:", F_tot)
    return F_nlg_allow/F_tot


def v_t(a,t):
    v = [0]
    for i in range(1,len(a)):
        v.append(a[i]*(t[i]-t[i-1]) + v[i-1])
    return v


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
    Roll_fric_car   = 0.0065


    N_mlg   =  m_plane*weight_ratio*9.81
    N_mlg_w =  N_mlg/4
    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81

    F_tot = m_tot*a + Roll_fric*N_mlg + Roll_fric_car*N_nlg  # [N] Total force req to move plane at acceleration
    F_mlg = (1-ratio)*F_tot  # [N] Force needed  from internal
    F_mlg_w = F_mlg/pow_wheel  # [N] Force needed  from internal per wheel

    T_mlg_w = F_mlg_w*w_rad_air

    if stat_traction(T_mlg_w, N_mlg_w, w_rad_air):
        print("EGTS: Static friction checked")
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
    Roll_fric_car   = 0.0065

    N_mlg   =  m_plane*weight_ratio*9.81
    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81
    N_nlg_w = N_nlg/4

    F_tot = m_tot*a + Roll_fric*N_mlg + Roll_fric_car*N_nlg  # [N] Total force req to move plane at acceleration
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
    return T_nlg_w_1*w_1, T_nlg_w_2*w_2


def s_v_a_plotter(title,time, power, velocity, acceleration):
    """
    :param title: title of plots
        'egts': On aircraft power
        'car': Car power
        ' ': No title
    :param time: time array
    :param power: power array
    :param velocity: velocity array
    :param acceleration: acceleration array
    :return: nothing
    """

    gs = gridspec.GridSpec(2, 2)

    fig = plt.figure()
    if title is 'egts':
        fig.suptitle("On Aircraft Power")

        powermax = max(power)
        time_idx = power.index(powermax)
        powermax = powermax/1000
        timemax = time[time_idx]

        ax1 = fig.add_subplot(gs[1, :])
        ax1.set_title("Power/Gear")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Power [kW]")
        ax1.plot(time, [i/1000 for i in power])
        ax1.annotate("max {pow}".format(pow=round(powermax, 2)), xy=(timemax, powermax), xytext=(timemax, powermax-100),
                     arrowprops=dict(facecolor='black', shrink=0.05, width=0.5, headwidth=9),)
        ax1.plot(time, [52 for i in time], color='grey', linestyle='--')


    elif title is 'car':
        fig.suptitle("Vehicle Power")

        powermax_0 = max(power[0,:])
        time_idx_0 = power[0,:].argmax()
        powermax_0 = powermax_0/1000
        timemax_0 = time[time_idx_0]

        ax0 = fig.add_subplot(gs[1, 0])
        ax0.set_title("Power Front Wheel")
        ax0.set_xlabel("Time [s]")
        ax0.set_ylabel("Power [kW]")
        ax0.plot(time, [i/1000 for i in power[0, :]])
        ax0.annotate("max {pow}".format(pow=round(powermax_0, 2)), xy=(timemax_0, powermax_0), xytext=(timemax_0,
                                            powermax_0-75), arrowprops=dict(facecolor='black',
                                                shrink=0.05, width=0.5, headwidth=9), )

        powermax_1 = max(power[1, :])
        time_idx_1 = power[1,:].argmax()
        powermax_1 = powermax_1/1000
        timemax_1 = time[time_idx_1]

        ax1 = fig.add_subplot(gs[1, 1])
        ax1.set_title("Power Rear Wheel")
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Power [kW]")
        ax1.plot(time, [i/1000 for i in power[1, :]])
        ax1.annotate("max {pow}".format(pow=round(powermax_1, 2)), xy=(timemax_1, powermax_1), xytext=(timemax_1,
                                            powermax_1-75), arrowprops=dict(facecolor='black',
                                                width=0.5, headwidth=9), )

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


def total_powerplot(P_nlg_tot, P_mlg_tot):
    fig = plt.figure()
    N = 3

    preheat = 92   # [kW] Engine pre-heating
    airco   = 125  # [kW] External Airco
    startup = 0    # [kW] Start-up
    steer   = 8    # [kW] Steering System
    sensor  = 2    # [kW] Computers/sensors

    Onaircraft = [np.array([0, P_nlg_tot + preheat+startup+sensor/2, 0]), 'Internal']
    EGTS       = [np.array([P_nlg_tot, 0, P_nlg_tot]), 'EGTS']
    P_car_prop = [np.array([0, P_mlg_tot, 0]), 'Car Tow']
    Pre_heat   = [np.array([preheat, 0, 0]), 'Pre-heating']
    Airco_ex   = [np.array([0, airco, 0]), 'External Airco']
    Start_up   = [np.array([startup, 0, 0]), 'Start-up']
    Steer_ex   = [np.array([0, steer, 0]), 'Steering System']
    Sensors    = [np.array([sensor/2, sensor/2, sensor/2]), 'Computer/sensors']

    ind = np.arange(N)    # the x locations for the groups
    width = 0.35       # the width of the bars: can also be len(x) sequence

    p1 = plt.bar(ind, Onaircraft[0], width)
    p2 = plt.bar(ind, EGTS[0], width, bottom=Onaircraft[0])
    p3 = plt.bar(ind, P_car_prop[0], width, bottom=EGTS[0]+Onaircraft[0])
    p4 = plt.bar(ind, Pre_heat[0], width, bottom=Onaircraft[0]+EGTS[0]+P_car_prop[0])
    p5 = plt.bar(ind, Airco_ex[0], width, bottom=Onaircraft[0]+EGTS[0]+P_car_prop[0]+Pre_heat[0])
    p6 = plt.bar(ind, Start_up[0], width, bottom=Onaircraft[0]+EGTS[0]+P_car_prop[0]+Pre_heat[0]+Airco_ex[0])
    p7 = plt.bar(ind, Steer_ex[0], width,
                 bottom=Onaircraft[0]+EGTS[0]+P_car_prop[0]+Pre_heat[0]+Airco_ex[0]+Start_up[0])
    p8 = plt.bar(ind, Sensors[0], width,
                 bottom=Onaircraft[0]+EGTS[0]+P_car_prop[0]+Pre_heat[0]+Airco_ex[0]+Start_up[0]+Steer_ex[0])

    p9 = plt.bar(ind, [0, 0, 62], width, facecolor='darkorange', edgecolor='gray', lw=2, ls='--')

    max_bar = Onaircraft[0]+EGTS[0]+P_car_prop[0]+Pre_heat[0]+Airco_ex[0]+Start_up[0]+Steer_ex[0]+Sensors[0]

    plt.annotate(round(max_bar[0], 2), xy=(ind[0], max_bar[0]), xytext=(ind[0]-0.15, max_bar[0]+10), )
    plt.annotate(round(max_bar[1], 2), xy=(ind[1], max_bar[1]), xytext=(ind[1]-0.15, max_bar[1]+10), )
    plt.annotate(round(max_bar[2], 2), xy=(ind[2], max_bar[2]), xytext=(ind[2]-0.15, max_bar[2]+10), )

    #plt.yticks(np.arange(0, 3001, 150))
    plt.ylabel('Power [kW]')
    plt.title('Power Usage Different Cases')
    plt.xticks(ind, ('Internal\n ICO \n External Power', 'External \n Vehicle', 'Internal \n ICO \n APU Power'))
    plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8[0], p9[0]),
               (Onaircraft[1], EGTS[1], P_car_prop[1], Pre_heat[1], Airco_ex[1], Start_up[1], Steer_ex[1], Sensors[1], "APU available"),
               loc='center left', bbox_to_anchor=(1., 0.5))
    fig.savefig('Total_Power_Sys_Dist', bbox_inches='tight')
    plt.show()
    pass


def static_power(velocity, time,ratio):
    P_plane, P_car_1, P_car_2 = [], [], []
    for vel in velocity:
        P_plane.append(EGTS_power(0, vel, ratio)/1000)
        i,j = car_power(0, vel, ratio)
        P_car_1.append(i/1000)
        P_car_2.append(j/1000)

    fig, axs = plt.subplots(4, sharex=True)
    fig.suptitle("Power Needed for Constant Velocity")
    axs[0].set_title("Velocity")
    axs[0].set_ylabel("Speed [m/s]")
    axs[0].plot(time,velocity,color='g')
    axs[1].set_title("Power EGTS/Gear")
    axs[1].set_ylabel("Power [kW]")
    axs[1].plot(time, P_plane)
    axs[2].set_title("Power Car Front Wheel")
    axs[2].set_ylabel("Power [kW]")
    axs[2].plot(time, P_car_1)
    axs[3].set_title("Power Car Rear Wheel")
    axs[3].set_ylabel("Power [kW]")
    axs[3].set_xlabel("Time [s]")
    axs[3].plot(time, P_car_2)
    plt.tight_layout()

    plt.show()
    pass


# ------------------------------------------------------- Inputs -------------------------------------------------------
F_nlg_allow = 120000
a = [1.8, 1.8, 1.75, 1.65, 1.4, 1.15, 1, 0.9, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.55, 0.5]

# ---------------------------------------------------- Calculations ----------------------------------------------------
t = [i for i in range((len(a)))]
v = v_t(a, t)

power_ratio = opt_force_ratio(F_nlg_allow, a)
print(" POWER RATIO EXTERNAL-INTERNAL: ", power_ratio)

P_egts_w = []
for i in range(len(a)):
    P_egts_w.append(EGTS_power(a[i], v[i], power_ratio))

P_car_w = np.zeros((2, len(a)))
for i in range(len(a)):
    P_car_w[0, i], P_car_w[1, i] = car_power(a[i], v[i], power_ratio)

s_v_a_plotter('egts', t, P_egts_w, v, a)
s_v_a_plotter('car', t, P_car_w, v, a)

total_powerplot((2*max(P_car_w[0, :])+2*max(P_car_w[1, :]))/1000, 2*max(P_egts_w)/1000)
static_power(v, t, power_ratio)
