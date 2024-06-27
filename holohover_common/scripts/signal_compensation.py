import os
import math
from ament_index_python.packages import get_package_share_directory
from holohover_common import HolohoverProps, Holohover, load_holohover_pros
import numpy as np
import matplotlib.pyplot as plt


def acc_to_sig(holohover: Holohover, state: np.ndarray, acc: np.ndarray):
    u_thrust = holohover.control_acceleration_to_force(state[:Holohover.NX], acc)
    u_signal = holohover.thrust_to_signal(u_thrust)
    return u_signal


def sig_to_acc(holohover: Holohover, state: np.ndarray, signal: np.ndarray):
    u_thrust = holohover.signal_to_thrust(signal)
    acc = holohover.control_force_to_acceleration(state[:Holohover.NX], u_thrust)
    return acc


def state_dynamics(props: HolohoverProps, holohover: Holohover, state: np.ndarray, u_signal: np.ndarray):
    x_dot = holohover.nonlinear_state_dynamics(state[:Holohover.NX], state[Holohover.NX:])
    x_dot_full = np.zeros(Holohover.NX + Holohover.NU)
    x_dot_full[:Holohover.NX] = x_dot
    x_dot_full[Holohover.NX:] = (u_signal - state[Holohover.NX:]) / props.motor_tau
    return x_dot_full


def integrate(props: HolohoverProps, holohover: Holohover, dt: float, state: np.ndarray, u_signal: np.ndarray):
    k1 = state_dynamics(props, holohover, state, u_signal)
    k2 = state_dynamics(props, holohover, state + dt * k1 / 2, u_signal)
    k3 = state_dynamics(props, holohover, state + dt * k2 / 2, u_signal)
    k4 = state_dynamics(props, holohover, state + dt * k3, u_signal)
    return state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def traj_no_comp(props, holohover, dt, steps, initial_state, v):

    x = np.zeros((steps + 1, Holohover.NX + Holohover.NU))
    acc = np.zeros((steps + 1, Holohover.NA))
    signal = np.zeros((steps, Holohover.NU))

    x[0, :] = initial_state
    acc[0, :] = sig_to_acc(holohover, x[0, :], x[0, Holohover.NX:])

    for i in range(steps):
        signal[i, :] = acc_to_sig(holohover, x[i, :], v[i, :])
        x[i + 1, :] = integrate(props, holohover, dt, x[i, :], signal[i, :])
        acc[i + 1, :] = sig_to_acc(holohover, x[i + 1, :], x[i + 1, Holohover.NX:])

    return x, acc, signal


def traj_comp(props, holohover, dt, steps, initial_state, v):

    x = np.zeros((steps + 1, Holohover.NX + Holohover.NU))
    acc = np.zeros((steps + 1, Holohover.NA))
    signal = np.zeros((steps, Holohover.NU))

    x[0, :] = initial_state
    acc[0, :] = sig_to_acc(holohover, x[0, :], x[0, Holohover.NX:])

    for i in range(steps):
        sig_min = props.idle_signal
        sig_max = 1.0
        f_min = holohover.signal_to_thrust(holohover.Ad_motor * x[i, Holohover.NX:] + holohover.Bd_motor * sig_min)
        f_max = holohover.signal_to_thrust(holohover.Ad_motor * x[i, Holohover.NX:] + holohover.Bd_motor * sig_max)
        u_thrust = holohover.control_acceleration_to_force(x[i, :Holohover.NX], v[i, :], f_min, f_max)
        signal[i, :] = holohover.thrust_to_signal(u_thrust)
        signal[i, :] = (signal[i, :] - holohover.Ad_motor * x[i, Holohover.NX:]) / holohover.Bd_motor
        x[i + 1, :] = integrate(props, holohover, dt, x[i, :], signal[i, :])
        acc[i + 1, :] = sig_to_acc(holohover, x[i + 1, :], x[i + 1, Holohover.NX:])

    return x, acc, signal


def main():
    time = 0.2
    dt = 0.01
    steps = math.ceil(time / dt)
    props = load_holohover_pros(os.path.join(get_package_share_directory('holohover_utils'),
                                             'config/common/holohover_params_new.yaml'))
    holohover = Holohover(props, dt)

    v = np.zeros((steps, Holohover.NA))  # acceleration set points
    v[:, 0] = 2.0  # acc in x direction

    initial_state = np.zeros(Holohover.NX + Holohover.NU)  # normal state + motor dynamics

    t = np.linspace(0, time, steps + 1)
    x_no_comp, acc_no_comp, signal_no_comp = traj_no_comp(props, holohover, dt, steps, initial_state, v)
    x_comp, acc_comp, signal_comp = traj_comp(props, holohover, dt, steps, initial_state, v)

    fig, ((ax0, ax1, ax2), (ax3, ax4, ax5), (ax6, ax7, ax8)) = plt.subplots(3, 3)

    ax0.plot(t, acc_no_comp[:, 0])
    ax0.plot(t, acc_comp[:, 0])
    ax0.set_ylabel("acc_x")
    ax0.grid()

    ax1.plot(t, acc_no_comp[:, 1])
    ax1.plot(t, acc_comp[:, 1])
    ax1.set_ylabel("acc_y")
    ax1.grid()

    ax2.plot(t, acc_no_comp[:, 2])
    ax2.plot(t, acc_comp[:, 2])
    ax2.set_ylabel("acc_yaw")
    ax2.grid()

    ax3.plot(t[:-1], signal_no_comp[:, 0])
    ax3.plot(t[:-1], signal_comp[:, 0])
    ax3.set_ylabel("signal_1")
    ax3.grid()

    ax4.plot(t[:-1], signal_no_comp[:, 1])
    ax4.plot(t[:-1], signal_comp[:, 1])
    ax4.set_ylabel("signal_2")
    ax4.grid()

    ax5.plot(t[:-1], signal_no_comp[:, 2])
    ax5.plot(t[:-1], signal_comp[:, 2])
    ax5.set_ylabel("signal_3")
    ax5.grid()

    ax6.plot(t[:-1], signal_no_comp[:, 3])
    ax6.plot(t[:-1], signal_comp[:, 3])
    ax6.set_ylabel("signal_4")
    ax6.grid()

    ax7.plot(t[:-1], signal_no_comp[:, 4])
    ax7.plot(t[:-1], signal_comp[:, 4])
    ax7.set_ylabel("signal_5")
    ax7.grid()

    ax8.plot(t[:-1], signal_no_comp[:, 5])
    ax8.plot(t[:-1], signal_comp[:, 5])
    ax8.set_ylabel("signal_6")
    ax8.grid()

    plt.show()


if __name__ == "__main__":
    main()
