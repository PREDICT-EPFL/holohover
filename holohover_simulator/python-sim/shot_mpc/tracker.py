"""
Tracking QP (Section 8): follows the selected plan at the control rate.

H nodes span the time left to the planned impact, so the grid rem / H shrinks
with it. The reference is the plan itself, propagated exactly with its ZOH
inputs onto the tracker nodes (Plan.state_at). The plan only has the
translational axes; the tracker also controls the heading and holds it at
theta_ref (Section 1).
The terminal constraint x_H = x_ref(t_f) is softened into a heavy terminal
cost, so the QP (input bounds only) is always feasible. The states are
condensed out: the only decision variables are U.
"""

import time                                                             # wall time of each solve

import numpy as np
import casadi as ca                                                     # symbolic QP and the DAQP interface

from .params import XY                                                  # (x, y, vx, vy) rows of the tracker state
from .model import zoh_coefficients                                     # exact ZOH map

NX, NU = 6, 3  # tracker state (x, y, theta, vx, vy, omega) and input (a_x, a_y, a_theta)


class Tracker:
    """Linear tracking MPC on the selected plan: one QP per control step, only its first input is applied."""

    def __init__(self, P, cfg):
        """P: ShotParams. cfg: the shot_mpc.tracker block of the configuration (H, t_min, theta_ref_world, Q, Q_f, R)."""
        self.P = P                                     # physics and limits (gamma, u_max, t_max)
        self.H = H = cfg["H"]                          # number of tracker intervals up to the impact
        self.t_min = cfg["t_min"]                      # below this time to impact, the plan input is used
        self.theta_ref = cfg["theta_ref_world"] + np.pi  # heading reference, planner frame (params.py)
        # Diagonal weights: Q on nodes 1..H-1, Q_f on node H (the impact), R on the inputs
        Q, Q_f, R = (np.array(cfg[key], dtype=float) for key in ("Q", "Q_f", "R"))
        umax = P.u_max                                 # input limits (a_x, a_y, a_theta), planner frame

        ###############
        ### Symbols ###
        ###############
        U = ca.SX.sym("U", NU, H)                      # decision variables: one input per interval
        x0 = ca.SX.sym("x0", NX)                       # parameter: measured robot state now
        Xr = ca.SX.sym("Xr", NX, H + 1)                # parameter: reference states at nodes 0..H
        Ur = ca.SX.sym("Ur", NU, H)                    # parameter: reference inputs of the intervals
        dt = ca.SX.sym("dt")                           # parameter: step length, time to impact / H
        # ZOH coefficients as expressions of dt. dt_max = t_max / H bounds dt and picks the
        # formula (the cancellation-free series when gamma * dt_max is small)
        e1, a, b = zoh_coefficients(dt, P.gamma, dt_max=P.t_max / H)

        ############
        ### Cost ###
        ############
        x, J = x0, 0                                   # rollout starts at the measured state, cost at 0
        for k in range(H):                             # interval k: node k -> node k + 1
            # Input deviation from the plan, normalised by the limits
            J += ca.sum1(R * ((U[:, k] - Ur[:, k]) / umax) ** 2)
            # Exact ZOH step: x becomes an expression of U, so the states are condensed out
            x = ca.vertcat(x[0:3] + a * x[3:6] + b * U[:, k], e1 * x[3:6] + a * U[:, k])
            # State error at node k + 1; the heavy Q_f at node H softens the terminal equality
            J += ca.sum1((Q_f if k == H - 1 else Q) * (x - Xr[:, k + 1]) ** 2)

        ##############
        ### Solver ###
        ##############
        # No constraints g: with the states condensed out, only the input bounds remain
        qp = {"x": ca.vec(U), "p": ca.vertcat(x0, ca.vec(Xr), ca.vec(Ur), dt), "f": J}
        # DAQP: dense active-set solver shipped with CasADi; failures are checked in solve()
        self.solver = ca.qpsol("tracker", "daqp", qp, {"error_on_fail": False})
        self.lbx = np.tile(-umax, H)                   # input bounds in the order of vec(U): u_0, u_1, ...
        self.ubx = np.tile(umax, H)
        self.times = []                                # wall time of every solve

    def solve(self, plan, t, x0):
        """First input (planner accelerations) of the tracking QP at absolute time t from robot state x0."""
        dt = plan.remaining(t) / self.H                         # tracker step: the horizon ends at the impact
        nodes = t + dt * np.arange(self.H + 1)                  # absolute times of nodes 0..H
        Xr = np.zeros((NX, self.H + 1))                         # reference states on the tracker nodes:
        Xr[XY] = plan.state_at(nodes, self.P.gamma)             # the plan on the translational axes,
        Xr[2] = self.theta_ref                                  # the heading held at theta_ref (omega = 0)
        Ur = np.zeros((NU, self.H))                             # reference inputs, a_theta = 0
        # Plan input active at the middle of each tracker interval (the grids do not align)
        Ur[0:2] = plan.input_at(nodes[:-1] + 0.5 * dt)
        # Parameters in the order of __init__; .T.ravel() is column-major, like ca.vec
        p = np.concatenate([x0, Xr.T.ravel(), Ur.T.ravel(), [dt]])

        t_start = time.perf_counter()                           # start timing
        # Initial guess: the plan inputs (CasADi's x0 is the guess, not the robot state)
        res = self.solver(x0=Ur.T.ravel(), p=p, lbx=self.lbx, ubx=self.ubx)
        self.times.append(time.perf_counter() - t_start)        # log the solve time
        if not self.solver.stats()["success"]:                  # QP failed:
            return Ur[:, 0]                                     # fall back to the plan input
        return np.array(res["x"]).ravel()[0:NU]                 # receding horizon: apply the first input only
