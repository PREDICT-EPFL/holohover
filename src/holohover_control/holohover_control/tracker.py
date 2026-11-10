"""
Tracking QP: follows a plan (plan.Plan) at the control rate, table frame.

H intervals span the time left to the planned impact, rem / H each, but never shorter than the
control period: an input is held for a whole period, so a shorter interval would let the QP
plan corrections the robot cannot do (near the impact it then chattered between the bounds).
When rem < H period the intervals are one period long, the one that reaches the impact is
shorter, and those after it are empty (the state stays at the impact).
The reference is the plan itself, propagated exactly with its ZOH inputs onto the tracker nodes
(Plan.state_at). The plan only has the translational axes; the tracker also controls the heading
and holds it at theta_ref.
The terminal constraint x_H = x_ref(t_f) is softened into a heavy terminal cost, so the QP (input
bounds only) is always feasible. The states are condensed out: the only decision variables are U.
solve() returns the whole input sequence (TrackerInputs): the controller applies its first input,
and replays it open loop once the impact time has passed.
It keeps no state between solves, so one instance serves every episode.
"""

from dataclasses import dataclass

import numpy as np
import casadi as ca                                                     # symbolic QP and the DAQP interface

from .plan import discrete_matrices, zoh_interval                       # exact ZOH map A_d, B_d

NX, NU = 6, 3  # tracker state (x, y, theta, vx, vy, omega) and input (a_x, a_y, a_theta)
XY = [0, 1, 3, 4]  # translational part (x, y, vx, vy) of the tracker state


@dataclass
class TrackerInputs:
    """Input sequence of one tracker solve, ZOH on the nodes t0 + k dt."""
    t0: float       # absolute time of node 0 (the solve time)
    dt: float       # tracker step, max(time to impact / H, period)
    U: np.ndarray   # (NU, H) inputs (a_x, a_y, a_theta)

    def input_at(self, t):
        """ZOH input at absolute time t (last input after the impact)."""
        return self.U[:, zoh_interval(np.asarray(t, dtype=float) - self.t0, self.dt, self.U.shape[1])]


class Tracker:
    """Linear tracking MPC on the plan: one QP per control step, only its first input is applied."""

    def __init__(self, gamma, acc_max, H, theta_ref, Q, Q_f, R, period):
        """gamma: viscous friction (1/s), the planner's. acc_max: input bounds (a_x, a_y, a_theta).
        H: intervals up to the impact. theta_ref: heading held while tracking. Diagonal weights on
        (x, y, theta, vx, vy, omega): Q on nodes 1..H-1, Q_f on node H (the impact); R on the input
        deviations from the plan, normalised by acc_max. period: s, the control period (the shortest
        interval)."""
        self.gamma = gamma
        self.H = H
        self.theta_ref = theta_ref
        self.period = period
        Q, Q_f, R, umax = (np.array(v, dtype=float) for v in (Q, Q_f, R, acc_max))

        ###############
        ### Symbols ###
        ###############
        U = ca.SX.sym("U", NU, H)                      # decision variables: one input per interval
        x0 = ca.SX.sym("x0", NX)                       # parameter: measured robot state now
        Xr = ca.SX.sym("Xr", NX, H + 1)                # parameter: reference states at nodes 0..H
        Ur = ca.SX.sym("Ur", NU, H)                    # parameter: reference inputs of the intervals
        dts = ca.SX.sym("dts", H)                      # parameter: interval lengths (0 after the impact)

        ############
        ### Cost ###
        ############
        x, J = x0, 0                                   # rollout starts at the measured state, cost at 0
        for k in range(H):                             # interval k: node k -> node k + 1
            # Input deviation from the plan, normalised by the limits
            J += ca.sum1(R * ((U[:, k] - Ur[:, k]) / umax) ** 2)
            # Exact ZOH step on (x, y, theta): x becomes an expression of U, so the states are condensed out
            A, B = discrete_matrices(dts[k], gamma, axes=3)
            x = A @ x + B @ U[:, k]
            # State error at node k + 1; the heavy Q_f at node H softens the terminal equality
            J += ca.sum1((Q_f if k == H - 1 else Q) * (x - Xr[:, k + 1]) ** 2)

        ##############
        ### Solver ###
        ##############
        # No constraints g: with the states condensed out, only the input bounds remain
        qp = {"x": ca.vec(U), "p": ca.vertcat(x0, ca.vec(Xr), ca.vec(Ur), dts), "f": J}
        # DAQP: dense active-set solver shipped with CasADi; failures are checked in solve()
        self.solver = ca.qpsol("tracker", "daqp", qp, {"error_on_fail": False})
        self.lbx = np.tile(-umax, H)                   # input bounds in the order of vec(U): u_0, u_1, ...
        self.ubx = np.tile(umax, H)

    def solve(self, plan, t, x0):
        """Input sequence of the tracking QP at absolute time t from the robot state x0
        (x, y, theta, vx, vy, omega)."""
        rem = plan.remaining(t)
        dt = max(rem / self.H, self.period)                     # tracker step, at least one control period
        dts = np.clip(rem - dt * np.arange(self.H), 0.0, dt)    # the horizon ends at the impact
        nodes = t + np.concatenate([[0.0], np.cumsum(dts)])     # absolute times of nodes 0..H
        Xr = np.zeros((NX, self.H + 1))                         # reference states on the tracker nodes:
        Xr[XY] = plan.state_at(nodes, self.gamma)               # the plan on the translational axes,
        Xr[2] = self.theta_ref                                  # the heading held at theta_ref (omega = 0)
        Ur = np.zeros((NU, self.H))                             # reference inputs, a_theta = 0
        # Plan input active at the middle of each tracker interval (the grids do not align)
        Ur[0:2] = plan.input_at(nodes[:-1] + 0.5 * dts)
        # Parameters in the order of __init__; .T.ravel() is column-major, like ca.vec
        p = np.concatenate([np.asarray(x0, dtype=float), Xr.T.ravel(), Ur.T.ravel(), dts])

        # Initial guess: the plan inputs (CasADi's x0 is the guess, not the robot state)
        res = self.solver(x0=Ur.T.ravel(), p=p, lbx=self.lbx, ubx=self.ubx)
        if not self.solver.stats()["success"]:                  # QP failed:
            return TrackerInputs(t, dt, Ur)                     # fall back to the plan inputs
        # vec(U) is column-major: reshape row by row into (H, NU), then transpose
        return TrackerInputs(t, dt, np.array(res["x"]).reshape(self.H, NU).T)
