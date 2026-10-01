"""
Tracking QP (Section 11): follows the selected plan at the control rate.

H nodes span the time left to the planned impact, so the grid rem / H shrinks
with it. The reference is the plan itself, propagated exactly with its ZOH
inputs onto the tracker nodes. The terminal constraint x_H = x_ref(t_f) is
softened into a heavy terminal cost, so the QP (input bounds only) is always
feasible. The states are condensed out: the only decision variables are U.
"""

import time                                                             # wall time of each solve

import numpy as np
import casadi as ca                                                     # symbolic QP and the DAQP interface

from .shot_nlp import NX, NU, zoh_coefficients                          # state/input sizes, exact ZOH map


def plan_reference(plan, times, gamma):
    """Plan states (NX, n) at absolute times, propagated from the preceding node with its ZOH input."""
    sol = plan.sol                                                      # plan solution
    N = sol.U.shape[1]                                                  # number of plan intervals (N + 1 nodes)
    s = np.asarray(times, dtype=float) - plan.t0                        # time since plan start
    k = np.clip(np.floor(s / plan.dt).astype(int), 0, N - 1)            # index of preceding plan node
    e1, a, b = zoh_coefficients(s - k * plan.dt, gamma, dt_max=plan.dt) # ZOH coefficients for each time
    X, U = sol.X[:, k], sol.U[:, k]                                     # plan states and inputs at preceding nodes
    # Exact ZOH step from node k: position p + a v + b u, velocity e1 v + a u
    return np.vstack([X[0:3] + a * X[3:6] + b * U, e1 * X[3:6] + a * U])


def plan_inputs(plan, times):
    """Plan ZOH inputs (NU, n) at absolute times."""
    N = plan.sol.U.shape[1]                                             # number of plan intervals
    # Plan interval containing each time; times at or after t_f get the last interval
    k = np.clip(np.floor((np.asarray(times) - plan.t0) / plan.dt).astype(int), 0, N - 1)
    return plan.sol.U[:, k]                                             # input held on that interval


class Tracker:
    """Linear tracking MPC on the selected plan: one QP per control step, only its first input is applied."""

    def __init__(self, P, cfg):
        """P: ShotParams. cfg: the shot_mpc.tracker block of the configuration (H, t_min, Q, Q_f, R)."""
        self.P = P                                     # physics and limits (gamma, u_max, t_max)
        self.H = H = cfg["H"]                          # number of tracker intervals up to the impact
        self.t_min = cfg["t_min"]                      # below this time to impact, the plan input is used
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
        Xr = plan_reference(plan, nodes, self.P.gamma)          # reference states on the tracker nodes
        # Plan input active at the middle of each tracker interval (the grids do not align)
        Ur = plan_inputs(plan, nodes[:-1] + 0.5 * dt)
        # Parameters in the order of __init__; .T.ravel() is column-major, like ca.vec
        p = np.concatenate([x0, Xr.T.ravel(), Ur.T.ravel(), [dt]])

        t_start = time.perf_counter()                           # start timing
        # Initial guess: the plan inputs (CasADi's x0 is the guess, not the robot state)
        res = self.solver(x0=Ur.T.ravel(), p=p, lbx=self.lbx, ubx=self.ubx)
        self.times.append(time.perf_counter() - t_start)        # log the solve time
        if not self.solver.stats()["success"]:                  # QP failed:
            return Ur[:, 0]                                     # fall back to the plan input
        return np.array(res["x"]).ravel()[0:NU]                 # receding horizon: apply the first input only
