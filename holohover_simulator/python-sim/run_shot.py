"""
Closed-loop test of the gradient-based shot MPC (air-hockey-mpc-formulation.typ)
in Till's simulator. Run from python-sim/:

    python run_shot.py --scenario stationary          # simulate + animate
    python run_shot.py --scenario incoming --export shot_incoming
    python run_shot.py --evaluate --episodes 50 --seed 0 [--no-jit] [--no-tracker]
    python run_shot.py --evaluate --episodes 50 --export eval [--export-outcomes miss]
    python run_shot.py --scenario incoming --mcap --no-animate   # Foxglove file, see foxglove_viz.py

The simulation loop is a plain Python loop (IPOPT cannot be traced by JAX) at the
control rate simulation.hz; each control step runs simulation.physics_hz / hz jitted
dynamics.physics_step substeps under the held input. Results are pickled with the
same keys as simulation.run_sim, so animation.TrajecoryAnimation and main.animate work.
"""

import argparse
import json
import os
import pickle
import signal
from datetime import datetime
from functools import partial

import numpy as np
import jax
import jax.numpy as jnp

from dynamics import physics_step
from controller import build_pd_controller
from foxglove_viz import write_mcap
from shot_mpc.params import load_config, physics_from_config, ShotParams
from shot_mpc.bounce_geometry import sequences, label
from shot_mpc.shot_nlp import ShotNLP
from shot_mpc.shot_controller import ShotController, ATT

N_FUTURE = 60


#######################
### Physics         ###
#######################
def n_substeps(config):
    """Physics steps per control step. The simulator only detects a robot-puck contact at
    the end of a step, and takes the contact normal from the overlapping centres: at 100 Hz
    a fast strike overlaps by ~2 cm and the shot leaves several degrees off. Physics at
    500 Hz keeps that error below half a degree."""
    sim = config["simulation"]
    n = sim.get("physics_hz", sim["hz"]) / sim["hz"]
    assert n >= 1 and n == int(n), "simulation.physics_hz must be a multiple of simulation.hz"
    return int(n)


@partial(jax.jit, static_argnums=4)
def physics_substeps(state, control, physics, h, n):
    """n physics steps of length h under one held control; per-substep contact and goal flags."""
    def body(s, _):
        s, r1_col, _, has_scored, _, _ = physics_step(s, control, physics, h)
        return s, (r1_col, has_scored)

    state, (r1_cols, scored) = jax.lax.scan(body, state, None, length=n)
    return state, r1_cols, scored


#######################
### Initialisation  ###
#######################
def initial_state(config, scenario=None):
    """18-dim state [robot1, robot2, puck], each (x, y, theta, vx, vy, omega)."""
    c = config["shot_mpc"]
    r1 = config["robot"]["robot1"]["initial_position"] + config["robot"]["robot1"]["initial_velocity"]
    puck = config["puck"]["initial_position"] + config["puck"]["initial_velocity"]
    if scenario:
        sc = c["scenarios"][scenario]
        r1 = sc.get("robot1", r1)
        puck = sc.get("puck", puck)
    if c["opponent"] == "static":
        r2 = config["robot"]["robot2"]["initial_position"] + config["robot"]["robot2"]["initial_velocity"]
    else:  # "none": parked in a corner of the opponent end, never actuated
        r2 = [c["opponent_park"][0], c["opponent_park"][1], 0.0, 0.0, 0.0, 0.0]
    return np.array(r1 + r2 + puck, dtype=float)


def sample_initial_state(rng, config):
    """Random start for the evaluation: robot near home, puck stationary in our half or incoming."""
    c = config["shot_mpc"]
    ev = c["evaluation"]
    width, height = config["table"]["width"], config["table"]["height"]
    r1 = np.array([config["robot"]["robot1"]["defensive_position_rel"] * width, 0.5 * height, 0.0, 0, 0, 0])
    r1[:2] += rng.uniform(-ev["robot1_jitter"], ev["robot1_jitter"], 2)
    d_min = config["robot"]["radius"] + config["puck"]["radius"] + 0.05
    while True:
        y = rng.uniform(*ev["puck_y"])
        if rng.uniform() < ev["stationary_fraction"]:
            puck = np.array([rng.uniform(*ev["puck_x_stationary"]), y, 0.0, 0.0, 0.0, 0.0])
        else:
            speed = rng.uniform(*ev["speed"])
            heading = np.deg2rad(rng.uniform(*ev["heading_deg"]))  # around +x: toward our goal
            puck = np.array([rng.uniform(*ev["puck_x_moving"]), y, 0.0,
                             speed * np.cos(heading), speed * np.sin(heading), 0.0])
        if np.linalg.norm(puck[:2] - r1[:2]) > d_min:
            break
    r2 = np.array([c["opponent_park"][0], c["opponent_park"][1], 0.0, 0.0, 0.0, 0.0])
    return np.concatenate([r1, r2, puck])


###############
### Episode ###
###############
CANDIDATE_KEYS = ("cand_robot", "cand_shot", "cand_score", "cand_valid", "cand_chosen")
RECORD_KEYS = ("states", "inputs", "plans", "tfs", "modes", "scores", "future") + CANDIDATE_KEYS


class Episode:
    """One closed-loop run, advanced one control step at a time (run_episode here, run_live.py)."""

    def __init__(self, config, physics, P, nlp, init_state, verbose=True):
        self.ctrl = ShotController(config, physics, P, nlp, verbose=verbose)
        self.physics = physics
        self.hz = config["simulation"]["hz"]
        self.dt = 1.0 / self.hz
        self.n_sub = n_substeps(config)
        self.h = self.dt / self.n_sub
        self.F_max = np.array(config["simulation"]["u_limits"])

        self.r2_ctrl = None
        if config["shot_mpc"]["opponent"] == "static":
            rel = config["robot"]["robot2"]["defensive_position_rel"]
            home = jnp.array([rel * physics.table.width, 0.5 * physics.table.height, 0.0, 0.0, 0.0, 0.0])
            self.r2_ctrl = build_pd_controller(home, robot_id=2)

        self.state = np.asarray(init_state, dtype=float)
        self.score = np.zeros(2, dtype=int)
        self.i = 0  # control steps taken; the simulation time is i * dt
        self.outcome, self.t_goal, self.i_goal, self.contacts_at_goal = None, None, None, None  # first goal
        self.attacked = False

    def step(self, record=True):
        """One control step, then simulation.physics_hz / hz physics substeps under the held input.
        With record, returns the step's row: one value per RECORD_KEYS key."""
        ctrl, state, i = self.ctrl, self.state, self.i
        t = i * self.dt
        F1 = ctrl.step(i, state)
        F2 = (np.clip(np.asarray(self.r2_ctrl(jnp.asarray(state)), dtype=float), -self.F_max, self.F_max)
              if self.r2_ctrl else np.zeros(3))
        self.attacked |= ctrl.mode == ATT

        row = None
        if record:
            row = {
                "inputs": F1,
                "plans": ctrl.plan_world(state),
                "tfs": max(ctrl.plan.remaining(t), 0.0) if ctrl.mode == ATT and ctrl.plan else 0.0,
                "modes": ctrl.mode,
            }
            fut = np.zeros((N_FUTURE, 18))
            fut[:, 6:9] = state[6:9]
            fut[:, 12:14] = ctrl.predicted_puck_world(state, t, N_FUTURE)
            row["future"] = fut
            row.update(zip(CANDIDATE_KEYS, ctrl.candidates_world()))

        u = jnp.asarray(np.concatenate([F1, F2]), dtype=jnp.float32)
        new_state, r1_cols, scored = physics_substeps(jnp.asarray(state, dtype=jnp.float32), u, self.physics,
                                                      self.h, self.n_sub)
        self.state = np.asarray(new_state, dtype=float)
        r1_cols, scored = np.asarray(r1_cols), np.asarray(scored)
        if r1_cols.any():
            ctrl.notify_contact(t + (np.argmax(r1_cols) + 1) * self.h)
        j_goal = np.flatnonzero(scored)
        has_scored = int(scored[j_goal[0]]) if len(j_goal) else 0
        if has_scored:
            self.score[has_scored - 1] += 1
            if self.outcome is None:
                self.outcome = "goal" if has_scored == 1 else "own_goal"
                self.t_goal, self.i_goal = t + (j_goal[0] + 1) * self.h, i
                self.contacts_at_goal = len(ctrl.contact_times)
        self.i += 1

        if record:
            row["states"] = self.state.copy()
            row["scores"] = self.score.copy()
        return row

    def final_outcome(self):
        """The first goal's outcome, or why there was none."""
        if self.outcome is not None:
            return self.outcome
        return "miss" if self.ctrl.contact_times else ("no_contact" if self.attacked else "no_attack")


def run_episode(config, physics, P, nlp, init_state, duration, stop_on_goal=False,
                verbose=True, record=True, tail=0.0):
    """With stop_on_goal, the episode ends `tail` seconds after the first goal."""
    ep = Episode(config, physics, P, nlp, init_state, verbose=verbose)
    tail_steps = int(round(tail * ep.hz))
    rec = {k: [] for k in RECORD_KEYS}

    for _ in range(int(round(duration * ep.hz))):
        row = ep.step(record)
        if record:
            for key, value in row.items():
                rec[key].append(value)
        if stop_on_goal and ep.i_goal is not None and ep.i - 1 >= ep.i_goal + tail_steps:
            break

    ctrl = ep.ctrl
    result = {
        "outcome": ep.final_outcome(),
        "t_goal": ep.t_goal,
        "contacts": list(ctrl.contact_times),
        "contacts_at_goal": ep.contacts_at_goal,
        "plan_history": ctrl.plan_history,
        "selector_logs": ctrl.selector.logs,
        "tracker_times": ctrl.tracker.times if ctrl.tracker else [],
    }
    if record:
        result["data"] = _animation_data(config, physics, init_state, rec, P.N)
    return result


def _animation_data(config, physics, init_state, rec, N):
    """Same keys and shapes as simulation.run_sim (row 0 = initial state)."""
    assert config["simulation"]["H"] == N, "simulation.H must equal shot_mpc.N for the animation"
    first = lambda key: np.concatenate([np.asarray(rec[key][:1]), np.asarray(rec[key])])
    n = len(rec["states"]) + 1

    # The animation rebuilds its time axis as np.arange(0, duration + dt, dt): give it the
    # recorded length (episodes can stop early), nudged so the float arange has n samples.
    dt = 1.0 / config["simulation"]["hz"]
    duration = (n - 1) * dt
    if len(np.arange(0, duration + dt, dt)) != n:
        duration -= 1e-9
    assert len(np.arange(0, duration + dt, dt)) == n
    config = {**config, "simulation": {**config["simulation"], "duration": duration}}
    return {
        "states": np.vstack([init_state[None, :], np.asarray(rec["states"])]),
        "input_sequence": first("inputs"),
        "path_plans": first("plans"),
        "tfs": first("tfs"),
        "modes": first("modes"),
        "scores": np.vstack([np.zeros((1, 2), dtype=int), np.asarray(rec["scores"])]),
        "future_trajs": first("future"),
        "diffusion_trajs": np.zeros((n, 1, 1, N, 2)),
        "diffusion_costs": np.ones((n, 1, 1)),
        "physics": physics,
        "config": config,
        # Every bounce sequence's latest solution, drawn semi-transparent by the animation
        "candidates": {
            "labels": [label(s, k) for s, k in sequences(config["shot_mpc"]["K"])],
            "robot": first("cand_robot"),
            "shot": first("cand_shot"),
            "score": first("cand_score"),
            "valid": first("cand_valid"),
            "chosen": first("cand_chosen"),
        },
    }


#################
### Reporting ###
#################
def timing_summary(selector_logs, tracker_times=()):
    solves = [r for log in selector_logs for r in log.results.values()]
    warm = np.array([r[4] for r in solves if not r[6]]) * 1e3
    cold = np.array([r[4] for r in solves if r[6]]) * 1e3
    outer = np.array([log.t_total for log in selector_logs if log.outer]) * 1e3
    inner = np.array([log.t_total for log in selector_logs if not log.outer]) * 1e3
    iters_warm = np.array([r[5] for r in solves if not r[6]])
    tracker = np.array(tracker_times) * 1e3

    def row(name, a):
        if len(a) == 0:
            return f"  {name:<22}      -"
        return (f"  {name:<22} n={len(a):<5d} mean={a.mean():7.2f}  p95={np.percentile(a, 95):7.2f}  "
                f"max={a.max():7.2f} ms")

    lines = ["Solve times:", row("one B, warm start", warm), row("one B, cold (all seeds)", cold),
             row("outer update (all B)", outer), row("inner update (B*)", inner),
             row("tracker QP", tracker)]
    if len(iters_warm):
        lines.append(f"  warm IPOPT iterations: mean {iters_warm.mean():.1f}, max {iters_warm.max()}")
    return "\n".join(lines)


def update_json(log):
    """One selector update (ShotSelector.logs entry) as plain data."""
    return {"t": log.t, "outer": log.outer, "chosen": log.chosen, "t_total": log.t_total,
            "results": {k: {"valid": v[0], "cost": v[1], "score": v[2], "tf": v[3],
                            "t_wall": v[4], "iters": v[5], "cold": v[6]} for k, v in log.results.items()}}


def shot_log_json(result):
    return {"outcome": result["outcome"], "t_goal": result["t_goal"], "contacts": result["contacts"],
            "plan_history": result["plan_history"], "updates": [update_json(log) for log in result["selector_logs"]]}


############
### Main ###
############
def simulate(config, physics, P, nlp, scenario, export=None, animate=True, save=True, mcap=False):
    init = initial_state(config, scenario)
    result = run_episode(config, physics, P, nlp, init, config["simulation"]["duration"])

    shots = [h[1] for h in result["plan_history"]]
    print(f"\nOutcome: {result['outcome']}"
          + (f" at t={result['t_goal']:.2f}s" if result["t_goal"] else "")
          + f" | contacts at {[round(c, 2) for c in result['contacts']]}"
          + f" | shots planned: {sorted(set(shots))}")
    print(timing_summary(result["selector_logs"], result["tracker_times"]))

    data = result["data"]
    now = datetime.now()
    base = f"simulation_data/shot_{scenario or 'default'}_{now.strftime('%Y%m%d_%H%M%S')}"
    if save:
        os.makedirs("simulation_data", exist_ok=True)
        path = base + ".pkl"
        with open(path, "wb") as f:
            pickle.dump(data, f)
        with open(path.replace(".pkl", "_log.json"), "w") as f:
            json.dump(shot_log_json(result), f, indent=1, default=float)
        print(f"Saved to {path}")
    if mcap:
        write_mcap(data, base + ".mcap", log=shot_log_json(result),
                   metadata={"scenario": scenario or "default", "outcome": result["outcome"]},
                   start_ns=int(now.timestamp() * 1e9))
        print(f"Saved Foxglove MCAP to {base}.mcap")

    if export:
        export_gif(data, gif_name(export))
    elif animate:
        play_animation(data)
    return result


def play_animation(data):
    """Interactive window; Ctrl+C closes it like its close button. A KeyboardInterrupt would
    mostly land in a Tk redraw callback, where tkinter prints it and the animation carries on,
    so the handler queues the close on the GUI event loop, which play() pumps every frame."""
    # Qt's default xcb plugin aborts on this GNOME/Wayland machine (see SP_hockey/README.md)
    if os.environ.get("XDG_SESSION_TYPE") == "wayland":
        os.environ.setdefault("QT_QPA_PLATFORM", "wayland")
    import matplotlib.pyplot as plt
    from animation import TrajecoryAnimation

    timers = []  # a backend timer garbage-collected before it fires never fires

    def close_on_sigint(signum, frame):
        if not plt.get_fignums():
            raise KeyboardInterrupt
        timer = plt.figure(plt.get_fignums()[0]).canvas.new_timer(interval=0)
        timer.single_shot = True
        timer.add_callback(plt.close, "all")
        timer.start()
        timers.append(timer)

    previous = signal.signal(signal.SIGINT, close_on_sigint)
    try:
        TrajecoryAnimation(**data).play(mode="standard")
    finally:
        signal.signal(signal.SIGINT, previous)


def gif_name(export):
    """animation.export_animation always writes gifs/<name>.gif: accept 'x', 'x.gif' or 'gifs/x.gif'."""
    name = os.path.basename(export)
    return name[:-4] if name.endswith(".gif") else name


def export_gif(data, name):
    import matplotlib

    matplotlib.use("Agg")  # no window needed to render a GIF; must precede the pyplot import
    import matplotlib.pyplot as plt
    from animation import TrajecoryAnimation

    anim = TrajecoryAnimation(**data)
    anim.export_animation(filename=name)
    plt.close(anim.fig)


def evaluate(config, physics, P, nlp, episodes, seed, export=None, export_outcomes=None, mcap=False):
    """With export, every episode (or only those whose outcome is in export_outcomes) is
    recorded to gifs/<NAME>_ep<index>_<outcome>.gif; with mcap, to
    simulation_data/shot_eval_<stamp>_ep<index>_<outcome>.mcap."""
    rng = np.random.default_rng(seed)
    duration = config["shot_mpc"]["evaluation"]["duration"]
    counts, logs, tracker_times, first_shot = {}, [], [], 0
    record = export is not None or mcap
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    for e in range(episodes):
        init = sample_initial_state(rng, config)
        res = run_episode(config, physics, P, nlp, init, duration, stop_on_goal=True, verbose=False,
                          record=record, tail=0.5 if record else 0.0)
        counts[res["outcome"]] = counts.get(res["outcome"], 0) + 1
        first_shot += res["outcome"] == "goal" and res["contacts_at_goal"] == 1
        logs.extend(res["selector_logs"])
        tracker_times.extend(res["tracker_times"])
        print(f"episode {e:3d}: {res['outcome']:<10} contacts={len(res['contacts'])}")
        if record and (not export_outcomes or res["outcome"] in export_outcomes):
            if export:
                export_gif(res["data"], f"{gif_name(export)}_ep{e:03d}_{res['outcome']}")
            if mcap:
                write_mcap(res["data"], f"simulation_data/shot_eval_{stamp}_ep{e:03d}_{res['outcome']}.mcap",
                           log=shot_log_json(res),
                           metadata={"seed": seed, "episode": e, "outcome": res["outcome"]})

    goals = counts.get("goal", 0)
    print(f"\nConversion: {goals}/{episodes} = {100 * goals / episodes:.0f}% "
          f"(first-shot goals: {first_shot}) | outcomes: {counts}")
    print(f"JIT: {nlp.jit} | tracker: {config['shot_mpc']['tracker']['enabled']}")
    print(timing_summary(logs, tracker_times))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="config/config_shot.json")
    parser.add_argument("--scenario", default=None, help="stationary | incoming | around (default: config)")
    parser.add_argument("--export", default=None,
                        help="write gifs/<NAME>.gif instead of opening a window; with --evaluate, "
                             "one gifs/<NAME>_ep<i>_<outcome>.gif per episode")
    parser.add_argument("--export-outcomes", nargs="+", default=None,
                        help="with --evaluate --export/--mcap: only record these outcomes, e.g. miss no_contact")
    parser.add_argument("--mcap", action="store_true",
                        help="also write simulation_data/shot_*.mcap for the Foxglove web app (foxglove_viz.py); "
                             "with --evaluate, one per recorded episode")
    parser.add_argument("--no-animate", action="store_true")
    parser.add_argument("--no-save", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--episodes", type=int, default=50)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--no-jit", action="store_true")
    parser.add_argument("--no-tracker", action="store_true", help="apply the plan's inputs open loop")
    parser.add_argument("--opponent", choices=["none", "static"], default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    if args.no_jit:
        config["shot_mpc"]["jit"] = False
    if args.no_tracker:
        config["shot_mpc"]["tracker"]["enabled"] = False
    if args.opponent:
        config["shot_mpc"]["opponent"] = args.opponent
    physics = physics_from_config(config)
    P = ShotParams.from_config(config)
    nlp = ShotNLP(P)

    if args.evaluate:
        evaluate(config, physics, P, nlp, args.episodes, args.seed,
                 export=args.export, export_outcomes=args.export_outcomes, mcap=args.mcap)
    else:
        simulate(config, physics, P, nlp, args.scenario, export=args.export,
                 animate=not args.no_animate, save=not args.no_save, mcap=args.mcap)


if __name__ == "__main__":
    main()
