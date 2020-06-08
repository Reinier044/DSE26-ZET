import matplotlib.pyplot as plt
import numpy as np


def check_requirement(value, required_value, how='gt', by=None, name=''):
    """
    Parameters
    ----------
    value : int, float
        the value to compare to the required value
    required_value : int, float.
        the required value
    how : string, optional.
        gt: value must be greater than requirement
        lt: value must be less than requirement
        gte: value must be greater than or equal to requirement
        lte: value must be less than or equal to requirement
        gtb: value must be greater than requirement by an amount
        ltb: value must be less than requirement by an amount
        The default is 'gt'.
    by: int, float, optional
        for use with how= gtb and ltb
    name : string, optional
        the name of the requirement. The default is ''.

    Returns
    -------
    tuple
        name of requirement and a boolean stating if it is met.
    """

    if how == 'gt':
        if by is not None:
            print('dont specify a by value')
            return None
        meets = True if value > required_value else False
    elif how == 'lt':
        if by is not None:
            print('dont specify a by value')
            return None
        meets = True if value < required_value else False
    elif how == 'gte':
        if by is not None:
            print('dont specify a by value')
            return None
        meets = True if value >= required_value else False
    elif how == 'lte':
        if by is not None:
            print('dont specify a by value')
            return None
        meets = True if value <= required_value else False
    elif how == 'gtb':
        if by is None:
            print('specify the by value')
            return None
        meets = True if value - required_value > by else False
    elif how == 'ltb':
        if by is None:
            print('specify the by value')
            return None
        meets = True if required_value - value > by else False

    try:
        print(f'The{" " + name if name else ""} requirement' \
              + f' is{" not" if not meets else ""} met')
        return name, meets
    except:
        print('error occured, check inputs')
        return None, None


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


# example uses
if __name__ == '__main__':
    energy_consumption = 97
    current_energy_consumption = 100

    emissions = 62
    max_emissions = 60

    reqs = []

    tlreq06 = check_requirement(energy_consumption, current_energy_consumption,
                                how='lt', name='[ZET-SH-TLREQ-06]')
    fpe01 = check_requirement(emissions, max_emissions,
                              how='lte', name='[ZET-F-PE-01]')

    reqs.append(tlreq06)
    reqs.append(fpe01)

    # prints all unmet requirements
    print(list(req[0] for req in reqs if not req[1]))

    t = np.arange(0, 50, 0.5)
    s = np.random.rand(100)
    v = np.random.rand(100)
    a = np.random.rand(100)
    s_v_a_plotter(t, s, v, a)
