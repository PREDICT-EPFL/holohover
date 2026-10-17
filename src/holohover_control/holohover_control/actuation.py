"""
Accelerations to motor signals, with the hovercraft model of holohover_common (its Python bindings):
the actuation of control_lqr, step by step.

At each control period:
- the motor velocities are advanced through the first-order motor lag (Ad_motor, Bd_motor) under the
  last signal sent, and the state one period ahead is predicted with the thrust they give;
- the commanded acceleration, minus the disturbance estimate, is allocated to the six motors (the
  thrust allocation QP of control_acceleration_to_force), within the thrust the motors can reach in
  one period;
- the signal is the one that brings the motors to that thrust, clipped to [idle_signal, 1].

Model state order (holohover_common): x, y, v_x, v_y, yaw, w_z.
"""

import numpy as np


class Actuation:
    def __init__(self, props_file, period):
        """props_file: the hovercraft's holohover_props YAML file. period: s, the control period."""
        from holohover_common.bindings import Holohover, load_holohover_pros

        self.props = load_holohover_pros(props_file)
        self.model = Holohover(self.props, period)
        self.motor_velocities = np.zeros(self.model.NU)
        self.signal = np.zeros(self.model.NU)  # the last signal sent

    def step(self, state, disturbance, acc):
        """The motor signals (6) for the next period. state: (x, y, v_x, v_y, yaw, w_z); disturbance: the
        EKF's (dist_x, dist_y, dist_yaw); acc: the commanded (a_x, a_y, w_dot_z), table frame, or None for
        no acceleration, not even the disturbance compensation (balanced thrusts, not idle motors)."""
        m, idle = self.model, self.props.idle_signal
        self.motor_velocities = m.Ad_motor * self.motor_velocities + m.Bd_motor * self.signal
        acc_now = m.control_force_to_acceleration(state, m.signal_to_thrust(self.motor_velocities))
        state_next = m.Ad @ state + m.Bd @ acc_now

        acc_next = np.zeros(3) if acc is None else np.asarray(acc, dtype=float) - disturbance
        reachable = m.Ad_motor * self.motor_velocities
        force_min = m.signal_to_thrust(reachable + m.Bd_motor * idle * np.ones(m.NU))
        force_max = m.signal_to_thrust(reachable + m.Bd_motor * np.ones(m.NU))
        force = m.control_acceleration_to_force(state_next, acc_next, force_min, force_max)
        signal = (m.thrust_to_signal(force) - reachable) / m.Bd_motor
        self.signal = np.clip(signal, idle, 1.0)
        return self.signal
