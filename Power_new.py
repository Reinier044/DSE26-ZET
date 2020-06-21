"""
Created on TUES JUN 2 14:53:58 2020  @author: Willem
General notes:

The respective efficiencies are implemented as follows: All efficiecies up to the motor input power calculation are
implemented in the respective functions, i.e. efficiency of the gears and of the electric motor itself. The electric
efficiency is applied after the functions. As of now the efficiency of the power source is not yet included.

For the acceleration profile it is important to note that a[0] is not part of the profile and added for plotting reasons

"""
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
    if F_nlg_allow/F_tot > 1:
        return 1
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


def init_acc(a):
    """
    Checks that the initial force/acceleration is sufficient to start rolling. It needs to be 2.5-2 times higher than
    rolling friction force.
    :param a: List of acceleration profile [m/s^2] (note a[0] is not part of the profile and just for the plots)
    :return:True or False depending on if it suffices
    """
    Roll_fric = 0.02  # [-] Rolling friction coefficient of airplane wheels
    if a[1] >= 2.5*Roll_fric:
        return True
    else:
        return False

def EGTS_power_ring(a, v, ratio: float, pow_wheel: int = 2):
    """
    Function that calculates the (peak) power required by each MLG engine based upon the acceleration and speed.
    :param a: List of all the accelerations. [m/s^2]
    :param v: List of the corresponding velocities. [m/s]
    :param ratio: Ratio of (maximum) force that will be provided by the external vehicle. [-]
    :param pow_wheel: The number of powered MLG wheels. Automatically set to 2 which is the same as EGTS. [-]
    :return: Required power to put on each ring gear given the acceleration needed at a certain speed and
    corresponding torque
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

    T_mlg_w = F_mlg_w*w_rad_air  # [Nm] Torque for wheels

    # Rotational speed of wheels
    w = v/w_rad_air  # [rad/s] rotational speed wheel

    # Check if static friction is not exceeded
    if not stat_traction(T_mlg_w, N_mlg_w, w_rad_air):
        print("LOG: Acceleration: {a} \tTorque: {t}".format(a=a, t= T_mlg_w))
        raise ValueError("Exceeds Static friction")

    else:
        print("Static friction check EGTS: [{t}]".format(t=True), end='\r')

    return T_mlg_w*w, T_mlg_w


def EGTS_tor_rpm_pow(torque, power, velocity, GR):
    """
    Function that can be used for engine sizing, it shows the required torque and power at the different RPM for
    both the ring gear and engine. It also indicates performance of the existing engine.
    :param torque: List of torque required for each acceleration interval on the ring gear or driven gear. [Nm]
    :param power: List of power required for each acceleration interval on the ring gear or driven gear. [Nm]
    :param velocity: Velocity list corresponding to acceleration interval. [m/s]
    :param GR: Gearing ratio. [-]
    :return: RPM of MLG wheels
    """
    # Efficiencies
    n_gear = 0.9875  # Gear efficiency (torque loss -> power loss)
    amount_gears = 2
    n_emotor = 0.95  # Electricmotor efficiency (electrical loss - power loss)

    torque_out = (1/n_gear)**amount_gears*np.array(torque)  # [Nm] Required engine output torque
    power_out  = (1/n_gear)**amount_gears*np.array(power)   # [W] Corresponding engine output power
    power_in   = (1/n_emotor)*power_out                     # [W] Required engine input power

    # Translate velocity to
    w_rad_air = 1.27/2    # [m] wheel radius aircraft MLG wheels
    w = np.array(velocity)/w_rad_air  # [rad/s]
    RPM = w*60/(2*np.pi)


    # Existing engines baseline
    T_ENG_268 = np.array([[0, 2000, 3000, 4000, 4500], [500, 500, 490, 482, 479]])
    T_ENG_348 = np.array([[0, 1200, 2600, 3500, 4000], [900, 1000, 1000, 958.33, 941.66]])
    P_ENG_268 = np.array([[0, 2200, 2830+1/3, 3600, 4500], [0, 120, 150, 180, 200]])
    P_ENG_348 = np.array([[0, 3000, 3500, 4000], [0, 315, 350, 370]])

    gs = gridspec.GridSpec(2, 2)  # Define figure layout
    fig = plt.figure("Engine Performance Plane")
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
    ax1.plot(RPM*GR, np.array(torque_out)/GR, 'red')
    ax1.plot(T_ENG_268[0, :], T_ENG_268[1, :], 'darkgray', linestyle='--')
    ax = ax1.twinx()
    ax.set_ylabel("EMRAX 268 PEAK", color='darkgray')
    ax.tick_params(right=False, labelright=False)
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
    ax = ax3.twinx()
    ax.set_ylabel("EMRAX 268 PEAK", color='darkgray')
    ax.tick_params(right=False, labelright=False)

    #ax3.plot(P_ENG_268[0, :], P_ENG_268[1, :]*2, 'darkgray')
    #ax3.plot(P_ENG_348[0, :], P_ENG_348[1, :], 'g')
    ax3.plot(RPM*GR, power_in/1000)

    fig.tight_layout()
    fig.subplots_adjust(top=0.88)
    fig.savefig('Power_ENG_Plane', bbox_inches='tight')
    plt.show()
    return power_in, torque_out, RPM


def s_v_a_plotter_egts(time, power, velocity, acceleration):
    """
    Function that presents this required power in a required power profile diagram together with the respective
    acceleration and velocity diagrams. The maximum power is also indicated in that graph.
    :param time: List of time intervals. [s]
    :param power: List of the corresponding power per engine. [W]
    :param velocity: List of the corresponding velocities. [m/s]
    :param acceleration: List of all the accelerations. [m/s^2]
    :return: Plots showing speed, acceleration and required peak power
    """
    gs = gridspec.GridSpec(2, 2)  # Define figure layout

    fig = plt.figure()
    fig.suptitle("On Aircraft Power")

    # Find maximum
    powermax = max(power)  # [W] Max power
    time_idx = np.argmax(power)  # Index Time Location max
    powermax = powermax/1000   # [kW] Max power
    timemax = time[time_idx]  # [s] Time Location max

    ax1 = fig.add_subplot(gs[1, :])
    ax1.set_title("Input Power/Gear")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Power [kW]")
    ax1.plot(time, [i/1000 for i in power])
    ax1.annotate("max {pow}".format(pow=round(powermax, 2)), xy=(timemax, powermax), xytext=(timemax, powermax-100),
                 arrowprops=dict(facecolor='black', shrink=0.06, width=0.6, headwidth=9),)

    # Velocity graphs
    ax2 = fig.add_subplot(gs[0, 0])
    ax2.set_title("Velocity")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.yaxis.set_ticks_position('left')
    ax2.plot(time, velocity, color='g')
    ax2.set_yticks([0, 5, 10, 15])
    ax0 = ax2.twinx()
    ax0.plot(time, velocity, color='g')
    ax0.set_ylabel("Velocity [kts]")
    ax0.set_yticks(np.array([0, 5.144, 2*5.144, 3*5.144]))
    ax0.set_yticklabels(['0', '10', '20', '30'])

    # Acceleration graphs
    ax3 = fig.add_subplot(gs[0, 1])
    ax3.set_title("Acceleration")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Acceleration [$m/s^2$]")
    ax3.set_ylim(0, max(acceleration)+0.2)
    ax3.plot(time, acceleration, color='r')

    # Plot
    fig.tight_layout()
    fig.savefig('Power_Wheel_Plane', bbox_inches='tight')
    plt.show()
    pass


def car_power(a, v, ratio, pow_wheel=4):
    """
    Function that calculates the (peak) power required to power each car wheel based upon the acceleration and speed.
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
    n_hydrostat     = 1         # [-] Efficiency hydrostatic motor

    # Necessary force and Torque calculations
    N_mlg   =  m_plane*weight_ratio*9.81                    # [N] Total normal force on the MLG
    N_nlg   = (m_car + m_plane*(1-weight_ratio))*9.81       # [N] Total normal force on the car
    N_nlg_w = N_nlg/4                                       # [N] Normal force per MLG wheel

    F_tot = m_tot*a + Roll_fric*N_mlg + Roll_fric_car*N_nlg  # [N] Total force req to move plane at acceleration
    F_nlg = ratio*F_tot                                      # [N] Force needed  from internal
    F_nlg_w = F_nlg/pow_wheel                                # [N] Force needed  from internal per wheel

    T_nlg_w_1 = F_nlg_w*w_rad_car_1            # [Nm] Torque per front wheel
    T_nlg_w_2 = F_nlg_w*w_rad_car_2            # [Nm] Torque per rear wheel

    # Rotational speed of wheels
    w_1 = v/w_rad_car_1  # [rad/s] rotational speed wheel
    w_2 = v/w_rad_car_2  # [rad/s] rotational speed wheel

    # Check if static friction is not exceeded
    if not stat_traction(T_nlg_w_1, N_nlg_w, w_rad_car_1):
        print("LOG: Acceleration: {a} \tTorque: {t} \tWheel Radius: {r}".format(a=a, t=T_nlg_w_1, r=w_rad_car_1))
        raise ValueError("Exceeds Static friction")
    elif not stat_traction(T_nlg_w_2, N_nlg_w, w_rad_car_2):
        print("LOG: Acceleration: {a} \tTorque: {t} \tWheel Radius: {r}".format(a=a, t=T_nlg_w_2, r=w_rad_car_2))
        raise ValueError("Exceeds Static friction")
    else:
        print("\tStatic friction check rear wheels: [{t}]. Static friction checked front wheels: [{t}]."
              .format(t=True), end="\r")

    return 1/n_hydrostat*T_nlg_w_1*w_1, 1/n_hydrostat*T_nlg_w_2*w_2


def s_v_a_plotter(time, power, velocity, acceleration):
    """
    Function that presents this required power in a required power profile diagram together with the respective
    acceleration and velocity diagrams. The maximum power is also indicated in that graph.
    :param time: List of time intervals. [s]
    :param power: List of the corresponding power per engine. [W]
    :param velocity: List of the corresponding velocities. [m/s]
    :param acceleration: List of all the accelerations. [m/s^2]
    :return: Plots showing speed, acceleration and required peak power
    """
    gs = gridspec.GridSpec(2, 2)  # Define figure layout
    fig = plt.figure("Power Car Wheels")
    fig.suptitle("Vehicle Power")

    # Find maximum
    powermax_0 = max(power[0, :])    # [W] Max power
    time_idx_0 = power[0, :].argmax()  # Index Time Location max
    powermax_0 = powermax_0/1000  # [kW] Max power
    timemax_0 = time[time_idx_0]  # [s] Time Location max

    ax0 = fig.add_subplot(gs[1, 0])
    ax0.set_title("Input Power Front Wheel")
    ax0.set_xlabel("Time [s]")
    ax0.set_ylabel("Power [kW]")
    ax0.plot(time, [i/1000 for i in power[0, :]])
    ax0.annotate(" MAN \nD2862 \n   /4 ", xy=(0, 0), xytext=(35, 140), color='gray')
    ax0.annotate("max {pow}".format(pow=round(powermax_0, 2)), xy=(timemax_0, powermax_0), xytext=(timemax_0-6,
                                        powermax_0-75), arrowprops=dict(facecolor='black',
                                            shrink=0.05, width=0.5, headwidth=5), )
    ax0.plot(time, [160 for i in time], color='gray', linestyle='--')
    # Find maximum
    powermax_1 = max(power[1, :])  # [W] Max power
    time_idx_1 = power[1, :].argmax()  # Index Time Location max
    powermax_1 = powermax_1/1000  # [kW] Max power
    timemax_1 = time[time_idx_1]  # [s] Time Location max

    ax1 = fig.add_subplot(gs[1, 1])
    ax1.set_title("Input Power Rear Wheel")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Power [kW]")
    ax1.plot(time, [i/1000 for i in power[1, :]])
    ax1.annotate("max {pow}".format(pow=round(powermax_1, 2)), xy=(timemax_1, powermax_1), xytext=(timemax_1-6,
                                        powermax_1-75), arrowprops=dict(facecolor='black',
                                            width=0.5, headwidth=5), )
    ax1.plot(time, [160 for i in time], color='gray', linestyle='--')

    # Velocity graphs
    ax2 = fig.add_subplot(gs[0, 0])
    ax2.set_title("Velocity")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Velocity [m/s]")
    ax2.plot(time, velocity, color='g')
    ax2.set_yticks([0, 5, 10, 15])
    ax = ax2.twinx()
    ax.plot(time, velocity, color='g')
    ax.set_ylabel("Velocity [kts]")
    ax.set_yticks(np.array([0, 5.144, 2*5.144, 3*5.144]))
    ax.set_yticklabels(['0', '10', '20', '30'])

    # Acceleration graphs
    ax3 = fig.add_subplot(gs[0, 1])
    ax3.set_title("Acceleration")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Acceleration [$m/s^2$]")
    ax3.set_ylim(0, max(acceleration)+0.05)
    ax3.plot(time, acceleration, color='r')

    # Plot
    fig.tight_layout()
    fig.savefig('Power_Wheel_Car', bbox_inches='tight')
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
    # Efficiencies
    n_gear = 0.9875  # Gear efficiency (torque loss -> power loss)
    amount_gears = 2
    n_emotor = 0.95  # Electricmotor efficiency (electrical loss - power loss)

    P_plane_ring, P_car_1, P_car_2 = [], [], []
    T_egts = []
    # Power calculation
    for vel in velocity:
        P_plane_ring.append(EGTS_power_ring(0, vel, ratio)[0]/1000)
        T_egts.append(EGTS_power_ring(0, vel, ratio)[1])
        i, j = car_power(0, vel, ratio)
        P_car_1.append(i/1000)
        P_car_2.append(j/1000)
    # Diagram w 4  plots
    P_plane = (1/n_emotor)*(1/n_gear)**amount_gears*np.array(P_plane_ring)
    fig, axs = plt.subplots(4, sharex='row')
    fig.suptitle("Power Needed for Constant Velocity")
    axs[0].set_title("Velocity")
    axs[0].set_ylabel("Speed [m/s]")
    axs[0].plot(time, velocity, color='g')
    axs[0].set_yticks([0, 5, 10, 15])
    ax0 = axs[0].twinx()
    ax0.plot(time, velocity, color='g')
    ax0.set_ylabel("Velocity [kts]")
    ax0.set_yticks(np.array([0, 5.144, 2*5.144, 3*5.144]))
    ax0.set_yticklabels(['0', '10', '20', '30'])
    axs[1].set_title("Power EGTS/Gear")
    axs[1].set_ylabel("Power [kW]")
    axs[1].plot(time, P_plane)
    axs[1].plot(time,[67 for i in time], color='gray', linestyle='--')
    axs[1].annotate("  EMRAX 268\nHV AC CONT", xy=(0, 0), xytext=(33, 67), color='gray', rotation=90 )
    axs[1].set_ylim(top=100)
    axs[1].set_yticks([0, np.max(P_plane), 67, 100])
    axs[2].set_title("Power Car Front Wheel")
    axs[2].set_ylabel("Power [kW]")
    axs[2].plot(time, P_car_1)
    axs[2].set_yticks([0, 30, 60])
    axs[3].set_title("Power Car Rear Wheel")
    axs[3].set_ylabel("Power [kW]")
    axs[3].set_xlabel("Time [s]")
    axs[3].plot(time, P_car_2)
    axs[3].set_yticks([0, 30, 60])

    fig.tight_layout()
    fig.subplots_adjust(top=0.88)
    fig.savefig('Stat_Power_Mov', bbox_inches='tight')
    plt.show()
    return np.array(P_plane)*1000, np.array(P_car_1)*1000, np.array(P_car_2)*1000


def EGTS_only_perf(GR):
    """
    This function calculates the egts only performance based upon the available APU power. It also plots this
    performance.
    :param GR: Gear ratio between MLG wheel and MLG engine
    :return: a_acc_slow: List of accelerations [m/s2], F_acc: list of corresponding forces [N], v_slow: list of
    speeds [m/s], time: time intervals accelerations [s]
    """
    #Power available
    P_APU  = 62              # [kW] Available apu power
    P_sen  = 0               # [kW]
    P_comp = 0               # [kW]
    P_av_e   = (P_APU-P_sen-P_comp)*1000/2  # [W] APU power available per engine

    # Efficiencies powertrain
    n_circuit = 0.97
    n_gear = 0.9875  # Gear efficiency (torque loss -> power loss)
    amount_gears = 2
    n_emotor = 0.95  # Electricmotor efficiency (electrical loss - power loss)

    # Airplane characteristics
    w_rad_air       = 1.27/2    # [m] wheel radius aircraft MLG wheels
    m_plane         = 97400     # [kg] MRW
    weight_ratio    = 0.952     # [-] Landing gear weight distribution ratio
    Roll_fric       = 0.02      # [-] Rolling friction coefficient of airplane wheels

    # Engine output torque for available power at different RPM calculation
    P_av_e_out  = n_circuit*n_emotor*P_av_e   # [W] engine output power
    T_egts_w_em = np.array([500])             # [Nm] engine output torque

    v_slow = np.arange(0, 8.1, 0.1)  # [kts] Velocity range
    v_slow = v_slow*0.514444         # to m/s
    w_slow = v_slow/w_rad_air        # [rad/s] corresponding rotational speed wheels
    w_slow_eng = w_slow*GR           # [rad/s] corresponding rotational speed engine
    for i in range(1, len(w_slow_eng)):
        # Enough power hence full torque
        if P_av_e_out/w_slow_eng[i] > 500:
            T_egts_w_em = np.append(T_egts_w_em, [500])
        # in sufficient power hence less torque
        elif P_av_e_out/w_slow_eng[i] < 500 and P_av_e_out/w_slow_eng[i] > 0:
            T_egts_w_em = np.append(T_egts_w_em, [P_av_e_out/w_slow_eng[i]])
        # not enough power
        else:
            T_egts_w_em = np.add(T_egts_w_em, [0])

    # Torque en power @ wheels = engine * gear efficiency
    T_egts_w_r = n_gear**amount_gears*GR*T_egts_w_em  # [W] wheel power
    F_egts_w = T_egts_w_r/w_rad_air                   # [Nm] engine output torque

    # Resultant acceleration calculation
    # Determining friction for resultant acceleration calculation
    N_mlg   = m_plane*weight_ratio*9.81                # [N] Total normal force on the MLG
    N_mlg_w = N_mlg/4                                  # [N] Normal force per MLG wheel
    N_nlg   = m_plane*(1-weight_ratio)*9.81            # [N] Total normal force of car
    F_fric = Roll_fric*N_mlg + Roll_fric*N_nlg         # [N] Total force req to move plane at acceleration

    # Resultant force
    F_acc = 2*F_egts_w-F_fric  # [N]

    # Resultant acceleration
    a_acc_slow = F_acc/m_plane  # [m/s2]
    # Cut-off insignificant accelerations
    v_slow = v_slow[np.where(a_acc_slow >= 0.005)]
    a_acc_slow = a_acc_slow[np.where(a_acc_slow >= 0.005)]

    # Determine time intervals for velocity intervals w corresponding acceleration profile
    time = np.array([0])
    for i in range(1, len(v_slow)):
        time = np.append(time, [v_slow[i]/a_acc_slow[i]])

    # Plot
    gs = gridspec.GridSpec(2, 2)  # Define figure layout
    fig = plt.figure("EGTS Only Performance")
    fig.suptitle("               EGTS Only Performance \n              Pushback")

    # Pushback velocity
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.set_title("Velocity")
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Velocity [m/s]")
    ax1.plot(time[0:31], v_slow[0:31], color='g')
    ax1.set_yticks([0, 0.5, 1, 1.5])
    ax = ax1.twinx()
    ax.plot(time[0:31], v_slow[0:31], color='g')
    ax.set_ylabel("Velocity [kts]")
    ax.set_yticks(np.array([0, 0.5144, 2*0.5144, 3*0.5144]))
    ax.set_yticklabels(['0', '1', '2', '3'])
    # Pushback Acceleration graphs
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.set_title("Acceleration")
    ax2.set_xlabel("Time [s]")
    ax2.set_ylabel("Acceleration [$m/s^2$]")
    ax2.set_ylim(0, max(a_acc_slow)+0.2)
    ax2.plot(time[0:31], a_acc_slow[0:31], color='r')

    # Slow taxi title
    ax0 = fig.add_subplot(gs[1, :])
    ax0.axis('off')
    ax0.set_title("Slow Taxi", pad=20)
    # Slow taxi
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.set_title("Velocity")
    ax3.set_xlabel("Time [s]")
    ax3.set_ylabel("Velocity [m/s]")
    ax3.plot(time, v_slow, color='g')
    ax3.plot(time, [2.88 for i in time], color='gray', linestyle='--')
    ax3.set_yticks([0, 0.5, 1, 1.5, 2, 2.5, 3])
    ax = ax3.twinx()
    ax.set_ylabel("Velocity [kts]")
    ax.set_yticks(np.array([0, 0.5144, 2*0.5144, 3*0.5144,  4*0.5144,  5*0.5144,  6*0.5144]))
    ax.set_yticklabels(['0', '1', '2', '3', '4', '5', '6'])
    # Pushback Acceleration graphs
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.set_title("Acceleration")
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Acceleration [$m/s^2$]")
    ax4.set_ylim(0, max(a_acc_slow)+0.2)
    ax4.plot(time, a_acc_slow, color='r')

    # Plot & Save
    fig.tight_layout()
    fig.subplots_adjust(top=0.88)
    fig.savefig('EGTS_Only_Perf', bbox_inches='tight')
    plt.show()
    return a_acc_slow, F_acc, v_slow, time


def total_powerplot(P_nlg_tot, P_mlg_tot):
    """
    Function that presents the total force needed in the different coperation cases as a bar plot.
    :param P_nlg_tot: List with the required powers at different accelerations [W]
    :param P_mlg_tot: List with the required powers at different accelerations [W]
    :return: Nothing, just shows bar plot.
    """
    fig = plt.figure()
    N = 2  # numbers of bars

    # Other power components
    preheat = 92*2  # [kW] Engine pre-heating
    airco   = 125   # [kW] External Airco
    startup = 0   # [kW] Start-up
    steer   = 8     # [kW] Steering System
    sensor  = 2     # [kW] Computers/sensors

    # Setting up bars
    EGTS       = [np.array([P_mlg_tot, 30]), 'EGTS']
    P_car_prop = [np.array([P_nlg_tot, 0]), 'Car Tow']
    Pre_heat   = [np.array([preheat, 0]), 'Pre-heating']
    Airco_ex   = [np.array([airco, 0]), 'External Airco']
    Start_up   = [np.array([startup, 0]), 'Start-up']
    Steer_ex   = [np.array([steer, 0]), 'Steering System']
    Sensors    = [np.array([sensor, sensor/2]), 'Computer/sensors']

    # Bar location and size
    ind = np.arange(N)    # the x locations for the groups
    width = 0.35          # the width of the bars: can also be len(x) sequence


    # Plot
    p0 = plt.bar(ind, [0, 62], width, facecolor='white', edgecolor='gray', lw=3, ls='--')

    p1 = plt.bar(ind, EGTS[0], width, bottom=0)
    p2 = plt.bar(ind, Pre_heat[0], width, bottom=EGTS[0])
    p3 = plt.bar(ind, Start_up[0], width, bottom=EGTS[0]+Pre_heat[0])

    p10 = plt.bar(ind, P_car_prop[0], width, bottom=EGTS[0]+Pre_heat[0]+Start_up[0])
    p11 = plt.bar(ind, Steer_ex[0], width,
                 bottom=EGTS[0]+Pre_heat[0]+Start_up[0]+P_car_prop[0])
    p12 = plt.bar(ind, Sensors[0], width,
                 bottom=EGTS[0]+Pre_heat[0]+Start_up[0]+P_car_prop[0]+Steer_ex[0])
    p13 = plt.bar(ind, Airco_ex[0], width, bottom=EGTS[0]+Pre_heat[0]+Start_up[0]+P_car_prop[0]+Steer_ex[0]+Sensors[0])


    # Annotations
    max_bar = EGTS[0]+P_car_prop[0]+Pre_heat[0]+Airco_ex[0]+Start_up[0]+Steer_ex[0]+Sensors[0]

    plt.annotate(round(max_bar[0], 2), xy=(ind[0], max_bar[0]), xytext=(ind[0]-0.15, max_bar[0]+10), )
    plt.annotate(round(max_bar[1], 2), xy=(ind[1], max_bar[1]), xytext=(ind[1]-0.15, 62+10), )

    # plt.yticks(np.arange(0, 3001, 150))
    plt.ylabel('Power [kW]')
    plt.title('Power Usage Different Cases')
    plt.xticks(ind, ('Power\n ICO \n External VEH', 'External \n Vehicle', 'Power \n ICO \n APU Only'))
    plt.legend((p0[0], p1[0], p2[0], p10[0], p11[0], p12[0], p13[0]),
               ("APU available", EGTS[1], Pre_heat[1], P_car_prop[1], Steer_ex[1], Sensors[1], Airco_ex[1]),
               loc='center left', bbox_to_anchor=(1., 0.5))


    plt.annotate('For \nAircraft', (0.175, (EGTS[0]+Pre_heat[0])[0]/2),
                (0.4, (EGTS[0]+Pre_heat[0])[0]/2),
                ha="center", va="center",
                size=14,
                arrowprops=dict(arrowstyle='-[, widthB='+str((EGTS[0]+Pre_heat[0])[0]*0.0055),
                                shrinkA=5,
                                shrinkB=5,
                                fc="k", ec="k",
                                ),
                bbox=dict(boxstyle="square", fc="w"), rotation=-90)


    fig.savefig('Total_Power_Sys_Dist', bbox_inches='tight')
    plt.show()
    pass


# ------------------------------------------------------- Inputs -------------------------------------------------------
F_nlg_allow = 79000  # [N] Maximum allowed force in the NLG strut
gearing_ratio_EGTS = 19  # [-] Gearing ration for the EGTS drive train
gearing_ratio_car = 5    # [-] Gearing ration for the vehicle  drive train
n_circuit = 0.97

#  NOTE: a[0] is not part of the profile and just for the plots!!!!!
a = np.array([0.7,
              0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7,
              0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7,
              0.65, 0.65, 0.65, 0.65, 0.65, 0.65, 0.65, 0.65, 0.59, 0.59, 0.59, 0.59, 0.57, 0.57, 0.55, 0.55, 0.5,
              0.5, 0.5, 0.5, 0.5, 0.5, 0.47, 0.47, 0.47, 0.47, 0.45, 0.45, 0.43, 0.43, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
              0.4, 0.4, 0.37, 0.37, 0.37, 0.37, 0.37, 0.37, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.35, 0.32,
              0.32, 0.32, 0.32, 0.32, 0.32, 0.32, 0.32, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.27, 0.27, 0.27, 0.27,
              0.27, 0.27, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25])
# [m/s^2]Wanted acceleration for complete sys

step = 4  # chose the amount of intervals per second corresponding to the chosen acceleration profile

# ---------------------------------------------------- Calculations ----------------------------------------------------
t = np.arange(0, len(a)/step, 1/step)  # [s] List of time intervals
v = v_t(a, t)  # [m/s] Velocities corresponding to chosen acceleration profile

print(" Power Calculation ".center(120, '#'))
power_ratio = opt_force_ratio(F_nlg_allow, a)  # Find ratio based upon Allowable force

# Check Initial force is sufficient
print(" Initial Force check ".center(120, '-'))
print("\tAdequate Initial: [{t}]".format(t=init_acc(a)), end='\n')

# On aircraft system calculations
P_egts_w_r = []
T_egts_w_r = []

for i in range(len(a)):
    i, j = EGTS_power_ring(a[i], v[i], power_ratio)
    P_egts_w_r.append(i)
    T_egts_w_r.append(j)
print("EGTS Static friction Check".center(120, '-'))
print("\tStatic friction check EGTS: [{t}]".format(t=True), end='\n')

P_egts_w_em, T_egts_w_em, rpm = EGTS_tor_rpm_pow(T_egts_w_r, P_egts_w_r, v, gearing_ratio_EGTS)


# Vehicle system calculations
P_car_w = np.zeros((2, len(a)))
print("Vehicle Static friction Check".center(120, '-'))
for i in range(len(a)):
    P_car_w[0, i], P_car_w[1, i] = car_power(a[i], v[i], power_ratio)
print("\tStatic friction check rear wheels: [{t}].\tStatic friction checked front wheels: [{t}]."
      .format(t=True), end='\n \n')

# Acceleration related plots
s_v_a_plotter_egts(t, P_egts_w_em, v, a)
s_v_a_plotter(t, P_car_w, v, a)

total_powerplot((1/n_circuit)*(2*max(P_car_w[0, :])+2*max(P_car_w[1, :]))/1000, (1/n_circuit)*(2*max(P_egts_w_em)/1000))

# Coasting related plots
P_egts_w_stat, P_car_w_stat_1, P_car_w_stat_2 = static_power(v, t, power_ratio)

# ----------------------------------------------- Printing Performance -------------------------------------------------
print(" Entire System Performance Characteristics ".center(120, '#'))
print('\n \t\tMaximum Velocity: {v}\t Maximum Acceleration: {a}'.format(v=v[-1], a=max(a)))
print(' \t\t\t0 -> {v} in {t} seconds'.format(v=round(v[-1], 2), t=t[-1]))
print(' \t\t\tAcceleration greater or equal to jet taxi: 0 -> {v}'.format(v=round(v[np.where(a>=0.7)[0][-1]], 3)))
print("Arrays".center(120, '-'))
print("Acceleration: \n  \t", np.array2string(np.array(a), precision=15, separator=', '), "\n")
print("Velocity: \n  \t", np.array2string(np.array(v), precision=4, separator=', '), "\n")
print("RPM: \n  \t", np.array2string(np.array(rpm), precision=4, separator=', '), "\n")

P_acc = (1/n_circuit)*(np.array(P_egts_w_em)*2+P_car_w[0, :]*2+P_car_w[1, :]*2)/1000
print("Total PEAK Power for Acceleration: \n  \t {P}"
      .format(P=np.array2string(P_acc, precision=4, separator=', ')), end='\n \n')
print("Peak Power: {p} \n".format(p=np.max(P_acc)).center(120, ' '))
P_stat = (1/n_circuit)*(np.array(P_egts_w_stat)*2+np.array(P_car_w_stat_1)*2+np.array(P_car_w_stat_2)*2)/1000
print("Total Power for cte Velocity: \n  \t {P}"
      .format(P=np.array2string(P_stat, precision=4, separator=', ')))
# Print unique acceleration array
i = 1
idx = []
while i+1 < len(a):
    if a[i] != a[i+1]:
        idx.append(i)
    i += 1

acc = a[[0]+idx]
vel = np.array(v)[[0]+idx]
print("\n \nClean Acceleration: \n  \t", np.array2string(np.array(acc), precision=15, separator=', '), "\n")
print("Clean Velocity: \n  \t", np.array2string(np.array(vel), precision=4, separator=', '), "\n")


print(" EGTS Only Performance Characteristics ".center(120, '#'))

a_push, F_push, v_push, time_push = EGTS_only_perf(gearing_ratio_EGTS)

print('\n \t\tMaximum Velocity: {v}\t Maximum Acceleration: {a}'.format(v=v_push[-1], a=max(a_push)))
print(' \t\t\t0 -> {v} in {t} seconds'.format(v=round(v_push[31], 2), t=time_push[31]))
print(' \t\t\t0 -> {v} in {t} seconds'.format(v=round(v_push[51], 2), t=time_push[51]))
print(' \t\t\t0 -> {v} in {t} seconds'.format(v=round(v_push[-1], 2), t=time_push[-1]))

for a_p in a_push:
    # Check if static friction is not exceeded
    if not stat_traction(a_p*97400*1.27/2, 97400*9.81, 1.27/2):
        print("LOG: Acceleration: {a} \tTorque: {t}".format(a=a, t= T_egts_w_r))
        raise ValueError("Exceeds Static friction")

    else:
        print("Static friction check EGTS: [{t}]".format(t=True), end='\r')

print("EGTS Static friction Check".center(120, '-'))
print("\tStatic friction check EGTS: [{t}]".format(t=True), end='\n')


print(" Initial Force check ".center(120, '-'))
print("\tAdequate Initial: [{t}]".format(t=init_acc(a)), end='\n')
