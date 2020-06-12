import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
# ----------------------------------------------------- Functions ------------------------------------------------------


def opt_force_ratio(F_nlg_allow, a_lst):
    """
    This function finds the power ratio between the external vehicle and the mlg gear engines. For this the max
    allowable force on the nose landing gear strut is used.
    :param F_nlg_allow: Maximum allowed 'fatigue' force of the nowe landing gear [N]
    :param a_lst: List of all the accelerations, which is needed to find max acceleration which correlates to
                    max force  [m/s^2]
    :return: Ratio of (maximum) force that will be provided by the external vehicle.
    """
    m_plane         = 97400     # [kg] MRW
    m_car           = 22000     # [kg] Mass of external vehicle
    m_tot = m_plane + m_car     # [kg] Total mass
    Roll_fric       = 0.02      # [-] Rolling friction coefficient

    F_tot = m_tot*max(a_lst) + Roll_fric*m_tot*9.81  # [N] Total force req to move plane at max acceleration
    print("\nMax force: {F} N  - POWER RATIO EXTERNAL-INTERNAL: {rat}\n".format(F=F_tot, rat=F_nlg_allow/F_tot))
    return F_nlg_allow/F_tot


def v_t(a, t):
    """
    Function that calculates the velocity for each constant acceleration and corresponding time interval.
    More specifically v(t) = a*t + v_0 will be used.
    :param a: List of all the accelerations. [m/s^2]
    :param t: List of corresponding time intervals. [s]
    :return: List of the corresponding velocities. [m/s]
    """
    v = [0]  # [m/s]
    for i in range(1, len(a)):
        v.append(a[i]*(t[i]-t[i-1]) + v[i-1])
    return v


def stat_traction(torque, N, wheelrad, fric=1):
    """
    Function that checks that the static friction / tractive force is not exceeded for a given torque and static
    friction coefficient. If the torque is higher than the limit the wheel will slip.
    :param torque: Torque of the wheel. [Nm]
    :param N: Normal force resting on the wheel. [N]
    :param wheelrad: Radius of the wheel. [m]
    :param fric: Static friction coefficient. Automatically set to 1 for tire (both airplane and vehicle) on dry concrete, other
                 values may be used to mimic other runway conditions and other tire types. [-]
    :return: True if limit is not exceeded and False if it is.
    """
    if torque > (fric*N)*wheelrad:
        #print("Too much torque, will slip")
        return False
    return True


def EGTS_power(a, v, ratio: float, pow_wheel: int = 2):
    """
    Function that calculates the (peak) power required by each MLG engine based upon the acceleration and speed.
    :param a: List of all the accelerations. [m/s^2]
    :param v: List of the corresponding velocities. [m/s]
    :param ratio: Ratio of (maximum) force that will be provided by the external vehicle. [-]
    :param pow_wheel: The number of powered MLG wheels. Automatically set to 2 which is the same as EGTS. [-]
    :return: Required power to put on each ring gear given the acceleration needed at a certain speed and
    corresponding torque
    """
    w_rad_air       = 1.27/2    # [m] wheel radius aircraft MLG wheels
    g_rad_w         = 0.770/2   # [m] Ring gear size
    m_plane         = 97400     # [kg] MRW
    m_car           = 20000     # [kg] Mass of external vehicle
    m_tot = m_plane + m_car     # [kg] Total mass of the system
    weight_ratio    = 0.952     # [-] Landing gear weight distribution ratio
    Roll_fric       = 0.02      # [-] Rolling friction coefficient of airplane wheels
    Roll_fric_car   = 0.0065    # [-] Rolling friction coefficient of external vehicle wheels

    N_mlg   = m_plane*weight_ratio*9.81                # [N] Total normal force on the MLG
    N_mlg_w = N_mlg/4                                  # [N] Normal force per MLG wheel
    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81  # [N] Total normal force of car

    F_tot = m_tot*a + Roll_fric*N_mlg + Roll_fric_car*N_nlg  # [N] Total force req to move plane at acceleration
    F_mlg = (1-ratio)*F_tot                                  # [N] Force needed  from internal
    F_mlg_w = F_mlg/pow_wheel                                # [N] Force needed  from internal per wheel

    T_mlg_w = F_mlg_w*w_rad_air             # [Nm] Torque for wheels (note on outer radius)
    T_ringg_w = T_mlg_w*g_rad_w/w_rad_air   # [Nm] Torque for wheels on ring gear

    if stat_traction(T_mlg_w, N_mlg_w, w_rad_air):
        print("EGTS: Static friction checked")
    else:
        raise ValueError("Exceeds Static friction")

    w = v/w_rad_air  # [rad/s] rotational speed wheel
    print("Main landing gear torque: {Tw} - Ring gear torque {Tg} - RPM {rpm}"
          .format(Tw=T_mlg_w, Tg=T_ringg_w, rpm=60/(2*np.pi)*w))
    return T_ringg_w*w, T_ringg_w


def car_power(a, v, ratio, pow_wheel=4):
    """
    Function that calculates the (peak) power required to power each car wheel based upon the acceleration and speed.
    :param a: List of all the accelerations. [m/s^2]
    :param v: List of the corresponding velocities. [m/s]
    :param ratio: Ratio of (maximum) force that will be provided by the external vehicle. [-]
    :param pow_wheel: The number of powered MLG wheels. Automatically set to 4 meaning the truck has to be 4WD [-].
    :return: Required power to put on each wheel given the acceleration needed at a certain speed.
    """
    w_rad_car_1     = 0.537     # [m] wheel radius front tires external truck
    w_rad_car_2     = 0.537     # [m] wheel radius rear tires external truck 0.496
    a_rad_w         = 0.5715/2  # [m] Axle radius
    m_plane         = 97400     # [kg] MRW
    m_car           = 20000     # [kg] Weight of external vehicle
    m_tot = m_plane + m_car     # [kg] Total mass of the system
    weight_ratio    = 0.952     # [-] Weight distribution ratio
    Roll_fric       = 0.02      # [-] Rolling friction coefficient for MLG gears
    Roll_fric_car   = 0.0065    # [-] Rolling friction coefficient for car wheels

    N_mlg   =  m_plane*weight_ratio*9.81                    # [N] Total normal force on the MLG
    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81       # [N] Total normal force on the car
    N_nlg_w = N_nlg/4                                       # [N] Normal force per MLG wheel

    F_tot = m_tot*a + Roll_fric*N_mlg + Roll_fric_car*N_nlg  # [N] Total force req to move plane at acceleration
    F_nlg = ratio*F_tot                                      # [N] Force needed  from internal
    F_nlg_w = F_nlg/pow_wheel                                # [N] Force needed  from internal per wheel

    T_nlg_w_1 = F_nlg_w*w_rad_car_1            # [Nm] Torque per front wheel (note on outer radius)
    T_a_w_1   = T_nlg_w_1*a_rad_w/w_rad_car_1  # [Nm] Torque per front axle
    T_nlg_w_2 = F_nlg_w*w_rad_car_2            # [Nm] Torque per rear wheel (note on outer radius)
    T_a_w_2   = T_nlg_w_2*a_rad_w/w_rad_car_2  # [Nm] Torque per rear axle

    # Check if static friction is not exceeded
    if stat_traction(T_nlg_w_1, N_nlg_w, w_rad_car_1):
        print("Static friction checked rear wheels.")
    else:
        raise ValueError("Exceeds Static friction")
    if stat_traction(T_nlg_w_2, N_nlg_w, w_rad_car_2):
        print("Static friction checked front wheels.")
    else:
        raise ValueError("Exceeds Static friction")
    w_1 = v/w_rad_car_1  # [rad/s] rotational speed wheel
    w_2 = v/w_rad_car_2  # [rad/s] rotational speed wheel
    return T_a_w_1*w_1, T_a_w_2*w_2


def s_v_a_plotter(title, time, power, velocity, acceleration):
    """
    Function that presents this required power in a required power profile diagram together with the respective
    acceleration and velocity diagrams. The maximum power is also indicated in that graph.
    :param title: title of plots
        'egts': On aircraft power
        'car': Car power
        ' ': No title
    :param time: List of time intervals. [s]
    :param power: List of the corresponding power per engine. [W]
    :param velocity: List of the corresponding velocities. [m/s]
    :param acceleration: List of all the accelerations. [m/s^2]
    :return: Plots showing speed, acceleration and required peak power
    """
    gs = gridspec.GridSpec(2, 2)  # Define figure layout
    fig = plt.figure()
    fig.suptitle("Vehicle Power")

    # Find maximum
    powermax_0 = max(power[0, :])    # [W] Max power
    time_idx_0 = power[0, :].argmax()  # Index Time Location max
    powermax_0 = powermax_0/1000  # [kW] Max power
    timemax_0 = time[time_idx_0]  # [s] Time Location max

    ax0 = fig.add_subplot(gs[1, 0])
    ax0.set_title("Power Front Wheel")
    ax0.set_xlabel("Time [s]")
    ax0.set_ylabel("Power [kW]")
    ax0.plot(time, [i/1000 for i in power[0, :]])
    ax0.annotate("max {pow}".format(pow=round(powermax_0, 2)), xy=(timemax_0, powermax_0), xytext=(timemax_0,
                                        powermax_0-75), arrowprops=dict(facecolor='black',
                                            shrink=0.05, width=0.5, headwidth=9), )
    # Find maximum
    powermax_1 = max(power[1, :])  # [W] Max power
    time_idx_1 = power[1, :].argmax()  # Index Time Location max
    powermax_1 = powermax_1/1000  # [kW] Max power
    timemax_1 = time[time_idx_1]  # [s] Time Location max

    ax1 = fig.add_subplot(gs[1, 1])
    ax1.set_title("Power Rear Wheel")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Power [kW]")
    ax1.plot(time, [i/1000 for i in power[1, :]])
    ax1.annotate("max {pow}".format(pow=round(powermax_1, 2)), xy=(timemax_1, powermax_1), xytext=(timemax_1,
                                        powermax_1-75), arrowprops=dict(facecolor='black',
                                            width=0.5, headwidth=9), )

    # Velocity graphs
    ax2 = fig.add_subplot(gs[0, 0])
    ax2.set_title("Velocity")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.plot(time, velocity, color='g')

    # Acceleration graphs
    ax3 = fig.add_subplot(gs[0, 1])
    ax3.set_title("Acceleration")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Acceleration [$m/s^2$]")
    ax3.set_ylim(0, max(acceleration)+0.2)
    ax3.plot(time, acceleration, color='r')

    # Plot
    plt.tight_layout()
    plt.show()
    pass


def s_v_a_plotter_egts(time, power, torque, velocity, acceleration):
    """
    Function that presents this required power in a required power profile diagram together with the respective
    acceleration and velocity diagrams. The maximum power is also indicated in that graph.
    :param time: List of time intervals. [s]
    :param power: List of the corresponding power per engine. [W]
    :param torque: List of the corresponding torques per engine [Nm]
    :param velocity: List of the corresponding velocities. [m/s]
    :param acceleration: List of all the accelerations. [m/s^2]
    :return: Plots showing speed, acceleration and required peak power
    """
    gs = gridspec.GridSpec(2, 2)  # Define figure layout

    fig = plt.figure()
    fig.suptitle("On Aircraft Power")

    # Find maximum
    powermax = max(power)  # [W] Max power
    time_idx = power.index(powermax)  # Index Time Location max
    powermax = powermax/1000   # [kW] Max power
    timemax = time[time_idx]  # [s] Time Location max

    ax1 = fig.add_subplot(gs[1, :])
    ax1.set_title("Power/Gear")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Power [kW]")
    ax1.plot(time, [i/1000 for i in power])
    ax1.annotate("max {pow}".format(pow=round(powermax, 2)), xy=(timemax, powermax), xytext=(timemax, powermax-100),
                 arrowprops=dict(facecolor='black', shrink=0.05, width=0.5, headwidth=9),)
    ax1.plot(time, [52 for i in time], color='grey', linestyle='--')

    # Velocity graphs
    ax2 = fig.add_subplot(gs[0, 0])
    ax2.set_title("Velocity")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.plot(time, velocity, color='g')

    # Acceleration graphs
    ax3 = fig.add_subplot(gs[0, 1])
    ax3.set_title("Acceleration")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Acceleration [$m/s^2$]")
    ax3.set_ylim(0, max(acceleration)+0.2)
    ax3.plot(time, acceleration, color='r')


    # Plot
    plt.tight_layout()
    plt.show()
    pass


def static_power(velocity, time, ratio):
    """
    Function that calucates continuous required power that is needed to overcome the rolling friction at a certain speed
    :param velocity: List of the corresponding velocities. [m/s]
    :param time: List of time intervals. [s]
    :param ratio: Ratio of (maximum) force that will be provided by the external vehicle. [-]
    :return: The diagram and the required power list @ each speed.
    """
    P_plane, P_car_1, P_car_2 = [], [], []
    T_egts = []
    # Power calculation
    for vel in velocity:
        P_plane.append(EGTS_power(0, vel, ratio)[0]/1000)
        T_egts.append(EGTS_power(0, vel, ratio)[1])
        i, j = car_power(0, vel, ratio)
        P_car_1.append(i/1000)
        P_car_2.append(j/1000)
    print(T_egts)
    # Diagram w 4  plots
    fig, axs = plt.subplots(4, sharex='row')
    fig.suptitle("Power Needed for Constant Velocity")
    axs[0].set_title("Velocity")
    axs[0].set_ylabel("Speed [m/s]")
    axs[0].plot(time, velocity, color='g')
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
    return P_plane*1000, P_car_1*1000, P_car_2*1000


def EGTS_tor_rpm_pow(torque, power, velocity):
    w_rad_air = 1.27/2    # [m] wheel radius aircraft MLG wheels
    w = np.array(velocity)/w_rad_air  # [rad/s]
    RPM = w*60/(2*np.pi)
    print('RPM', RPM)
    GR = 12
    TR = 16

    T_ENG_268 = np.array([[0, 2000, 3000, 4000, 4500], [500, 500, 490, 482, 479]])
    T_ENG_348 = np.array([[0, 1200, 2600, 3500, 4000], [900, 1000, 1000, 958.33, 941.66]])
    P_ENG_268 = np.array([[0, 2200, 2830+1/3, 3600, 4500], [0, 120, 150, 180, 200]])
    P_ENG_348 = np.array([[0, 3000, 3500, 4000], [0, 315, 350, 370]])

    gs = gridspec.GridSpec(2, 2)  # Define figure layout
    fig = plt.figure()
    fig.suptitle("Engine Required Acceleration Performance")

    ax0 = fig.add_subplot(gs[0, 0])
    ax0.set_title("Ring Gear")
    ax0.set_xlabel("RPM")
    ax0.set_ylabel("Torque [Nm]")
    ax0.plot(RPM, np.array(torque), 'red')
    ax1 = fig.add_subplot(gs[0, 1])
    ax1.set_title("Engine")
    ax1.set_xlabel("RPM")
    ax1.set_ylabel("Torque [Nm]")
    ax1.plot(RPM*GR, np.array(torque)/GR, 'red')
    ax1.plot(T_ENG_268[0, :], T_ENG_268[1, :], 'darkgray', linestyle='--')
    #ax1.plot(T_ENG_268[0, :], T_ENG_268[1, :]*2, 'darkgray')
    #ax1.plot(T_ENG_348[0, :], T_ENG_348[1, :], 'g')
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.set_title("Ring Gear")
    ax2.set_xlabel("RPM")
    ax2.set_ylabel("Power [kW]")
    ax2.plot(RPM, np.array(power)/1000)
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.set_title("Engine")
    ax3.set_xlabel("RPM")
    ax3.set_ylabel("Power [kW]")
    ax3.plot(P_ENG_268[0, :], P_ENG_268[1, :], 'darkgray', linestyle='--')
    #ax3.plot(P_ENG_268[0, :], P_ENG_268[1, :]*2, 'darkgray')
    #ax3.plot(P_ENG_348[0, :], P_ENG_348[1, :], 'g')
    ax3.plot(RPM*GR, np.array(power)/1000)

    plt.tight_layout()
    plt.show()
    pass


def EGTS_only_perf():
    P_APU     = 62              # [kW] Available apu power
    P_APU_eng = P_APU*1000/2    # [W] APU power available per engine



    return


def total_powerplot(P_nlg_tot, P_mlg_tot):
    """
    Function that presents the total force needed in the different coperation cases as a bar plot.
    :param P_nlg_tot: List with the required powers at different accelerations [W]
    :param P_mlg_tot: List with the required powers at different accelerations [W]
    :return: Nothing, just shows bar plot.
    """
    fig = plt.figure()
    N = 3

    # Other power components
    preheat = 92*2 # [kW] Engine pre-heating
    airco   = 125  # [kW] External Airco
    startup = 0    # [kW] Start-up
    steer   = 8    # [kW] Steering System
    sensor  = 2    # [kW] Computers/sensors

    # Setting up bars
    Onaircraft = [np.array([0, P_mlg_tot + preheat+startup+sensor/2, 0]), 'Internal']
    EGTS       = [np.array([P_mlg_tot, 0, P_mlg_tot]), 'EGTS']
    P_car_prop = [np.array([0, P_nlg_tot, 0]), 'Car Tow']
    Pre_heat   = [np.array([preheat, 0, 0]), 'Pre-heating']
    Airco_ex   = [np.array([0, airco, 0]), 'External Airco']
    Start_up   = [np.array([startup, 0, 0]), 'Start-up']
    Steer_ex   = [np.array([0, steer, 0]), 'Steering System']
    Sensors    = [np.array([sensor/2, sensor/2, sensor/2]), 'Computer/sensors']

    # Bar location and size
    ind = np.arange(N)    # the x locations for the groups
    width = 0.35          # the width of the bars: can also be len(x) sequence

    # Plot
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

    # plt.yticks(np.arange(0, 3001, 150))
    plt.ylabel('Power [kW]')
    plt.title('Power Usage Different Cases')
    plt.xticks(ind, ('Internal\n ICO \n External Power', 'External \n Vehicle', 'Internal \n ICO \n APU Power'))
    plt.legend((p1[0], p2[0], p3[0], p4[0], p5[0], p6[0], p7[0], p8[0], p9[0]),
               (Onaircraft[1], EGTS[1], P_car_prop[1], Pre_heat[1], Airco_ex[1], Start_up[1], Steer_ex[1], Sensors[1], "APU available"),
               loc='center left', bbox_to_anchor=(1., 0.5))

    fig.savefig('Total_Power_Sys_Dist', bbox_inches='tight')
    plt.show()
    pass


# ------------------------------------------------------- Inputs -------------------------------------------------------
F_nlg_allow = 115000  # [N] Maximum allowed force in the NLG strut
a = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
#[1.8, 1.8, 1.8, 1.62, 1.42, 1.16, 1.01, 0.92, 0.83, 0.77, 0.72, 0.67, 0.6, 0.57, 0.55, 0.53]  # Acceleration profile

# ---------------------------------------------------- Calculations ----------------------------------------------------
t = [i for i in range((len(a)))]
v = v_t(a, t)

power_ratio = opt_force_ratio(F_nlg_allow, a)

P_egts_w = []
T_egts_w = []
for i in range(len(a)):
    i, j = EGTS_power(a[i], v[i], power_ratio)
    P_egts_w.append(i)
    T_egts_w.append(j)

P_car_w = np.zeros((2, len(a)))
for i in range(len(a)):
    P_car_w[0, i], P_car_w[1, i] = car_power(a[i], v[i], power_ratio)

s_v_a_plotter_egts(t, P_egts_w, T_egts_w, v, a)
EGTS_tor_rpm_pow(T_egts_w, P_egts_w, v)
#s_v_a_plotter('car', t, P_car_w, v, a)

P_egts_w_stat, P_car_w_stat_1, P_car_w_stat_2 = static_power(v, t, power_ratio)
total_powerplot((2*max(P_car_w[0, :])+2*max(P_car_w[1, :]))/1000, 2*max(P_egts_w)/1000)

print("Total PEAK Power for Acceleration: \n  \t {P}".format(P=np.array(P_egts_w)*2+P_car_w[0,:]*2+P_car_w[1,:]*2))
print("Total Power for cte Velocity: \n  \t {P}"
      .format(P=np.array(P_egts_w_stat)*2+np.array(P_car_w_stat_1)*2+np.array(P_car_w_stat_2)*2))