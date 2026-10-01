"""
Live shot-MPC simulation for the Foxglove app: the closed loop of run_shot.py, streamed over a
WebSocket and paced to wall-clock time, at full speed or slowed down. Run from python-sim/:

    python run_live.py                                   # random starts, half speed
    python run_live.py --scenario incoming --speed 0.25
    python run_live.py --speed 1 --no-auto-restart

Open the printed app.foxglove.dev link (or "Open connection..." > Foxglove WebSocket >
ws://localhost:8765) and import the layout foxglove/shot_mpc_live_layout.json (foxglove/README.md).
The simulation starts when the app subscribes, so the first episode is seen from its start.

Controls in the app (the layout has a button for each service):
    /pause service     {"paused": true (default) | false}: pause or resume
    /step service      {"steps": <int>, default 1}: pause, then advance that many control steps
    /reset service     {"scenario": "random" | a config scenario, "seed": <int>}, both optional:
                       restart the current start, or switch to another one. Random draw k with
                       seed s is episode k of run_shot.py --evaluate --seed s.
    Parameters panel   speed (fraction of real time), paused, auto_restart (type the JSON value,
                       then click the check mark to send it)

With auto_restart, an episode ends one second after the first goal, or at its duration
(simulation.duration for a scenario, shot_mpc.evaluation.duration for a random start), and the
next one starts: the same scenario again, or the next random draw. Without it, play goes on
until the next /reset.

Simulated time is decoupled from wall time: every step is run_shot.Episode.step, as offline, so a
live episode is the same as the offline one. A step whose computation outlasts its wall-clock slot
(a cold outer solve takes up to ~300 ms) holds the view until it is done, and the simulation does
not fast-forward afterwards to catch up. /live/stats shows the speed reached and the compute time
of each step.

Topics: those of foxglove_viz.py, logged as the steps run, plus /live/stats. Log times follow the
simulated time of the whole session (a new episode continues the clock rather than restarting it);
the t field of each message is the time within its episode.
"""

import argparse
import json
import queue
import threading
import time
from collections import deque
from types import SimpleNamespace

import numpy as np

import foxglove
from foxglove import Capability, MessageSchema, Parameter, ParameterType, Schema, Service, ServiceSchema
from foxglove import messages as fm

from foxglove_viz import (MODE_NAMES, NUMBER, TRAIL_LENGTH, clear_overlays, geometry, json_channel,
                          log_bodies, log_event, log_static, log_step, setup_channels, solve_message)
from run_shot import Episode, initial_state, sample_initial_state, update_json
from shot_mpc.bounce_geometry import label, sequences
from shot_mpc.params import load_config, physics_from_config, ShotParams
from shot_mpc.shot_nlp import ShotNLP

SPEED_RANGE = (0.01, 4.0)  # fraction of real time
MAX_LAG = 0.1  # s of wall time behind schedule before the pacing stops trying to catch up
RESTART_AFTER_GOAL = 1.0  # s of simulated time
PAUSE_POLL = 0.02  # s
SUBSCRIBE_GRACE = 1.0  # s after the first subscription: the app subscribes panel by panel
STATIC_TOPICS = ("/scene/table", "/scene/bodies")
STATS_SCHEMA = {"t": NUMBER, "speed": NUMBER, "achieved_speed": NUMBER, "step_ms": NUMBER}


################
### Controls ###
################
MAX_STEPS = 1000  # per /step call
OK = json.dumps({"ok": True}).encode()  # service response


class Controls(foxglove.ServerListener):
    """What the app sets: parameters and service calls. The server thread writes, the sim loop reads."""

    def __init__(self, speed, auto_restart, scenarios):
        self._lock = threading.Lock()
        self._values = {"speed": speed, "paused": False, "auto_restart": auto_restart}
        self._steps = 0  # control steps requested by /step, run while paused
        self.scenarios = scenarios
        self.resets = queue.SimpleQueue()  # (scenario or None, seed or None)
        self.resend_static = threading.Event()
        self.subscribed = threading.Event()  # a client has subscribed to a topic
        self.params_changed = threading.Event()  # set by a service: the sim loop republishes the parameters

    def values(self):
        with self._lock:
            return dict(self._values)

    def take_step(self):
        """True if a /step control step is pending (and consume it)."""
        with self._lock:
            if self._steps == 0:
                return False
            self._steps -= 1
            return True

    def _parameters(self):
        v = self._values
        return [Parameter("speed", value=v["speed"], type=ParameterType.Float64),
                Parameter("paused", value=v["paused"]),
                Parameter("auto_restart", value=v["auto_restart"])]

    def parameters(self):
        with self._lock:
            return self._parameters()

    def _set_paused(self, paused):
        self._values["paused"] = paused
        if not paused:
            self._steps = 0

    def on_get_parameters(self, client, param_names, request_id=None):
        with self._lock:
            return [p for p in self._parameters() if not param_names or p.name in param_names]

    def on_set_parameters(self, client, parameters, request_id=None):
        with self._lock:
            for p in parameters:
                value = p.get_value()
                if p.name == "speed" and isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0:
                    self._values["speed"] = float(np.clip(value, *SPEED_RANGE))
                elif p.name == "paused" and isinstance(value, bool):
                    self._set_paused(value)
                elif p.name == "auto_restart" and isinstance(value, bool):
                    self._values[p.name] = value
            # An invalid value is answered with the current one
            return self._parameters()

    def on_subscribe(self, client, channel):
        self.subscribed.set()
        # The static scene is logged once: send it again to a client that joins later
        if channel.topic in STATIC_TOPICS:
            self.resend_static.set()

    ### Service handlers (the sim loop acts on them before its next step) ###
    def pause(self, request):
        paused = _request(request).get("paused", True)
        if not isinstance(paused, bool):
            raise ValueError("paused must be true or false")
        with self._lock:
            self._set_paused(paused)
        self.params_changed.set()
        return OK

    def step(self, request):
        steps = _request(request).get("steps", 1)
        if isinstance(steps, bool) or not isinstance(steps, int) or not 1 <= steps <= MAX_STEPS:
            raise ValueError(f"steps must be an integer from 1 to {MAX_STEPS}")
        with self._lock:
            self._set_paused(True)
            self._steps += steps
        self.params_changed.set()
        return OK

    def reset(self, request):
        args = _request(request)
        scenario, seed = args.get("scenario"), args.get("seed")
        if scenario not in (None, "random", *self.scenarios):
            raise ValueError(f"unknown scenario {scenario!r}; use random or one of {', '.join(self.scenarios)}")
        if seed is not None and (isinstance(seed, bool) or not isinstance(seed, int)):
            raise ValueError("seed must be an integer")
        self.resets.put((scenario, seed))
        return OK


def _request(request):
    args = json.loads(request.payload) if request.payload.strip() else {}
    if not isinstance(args, dict):
        raise ValueError("the request must be a JSON object")
    return args


def services(controls):
    """/pause, /step and /reset, all JSON."""
    def json_schema(name, properties):
        data = json.dumps({"type": "object", "properties": properties}).encode()
        return MessageSchema(encoding="json", schema=Schema(name=name, encoding="jsonschema", data=data))

    response = json_schema("holohover.LiveResponse", {"ok": {"type": "boolean"}})
    requests = {
        "/pause": ("holohover.Pause", {"paused": {"type": "boolean"}}, controls.pause),
        "/step": ("holohover.Step", {"steps": {"type": "integer"}}, controls.step),
        "/reset": ("holohover.Reset", {"scenario": {"type": "string"}, "seed": {"type": "integer"}}, controls.reset),
    }
    return [Service(name, schema=ServiceSchema(name=type_name, request=json_schema(type_name + "Request", properties),
                                               response=response), handler=handler)
            for name, (type_name, properties, handler) in requests.items()]


################
### Live sim ###
################
class LiveSim:
    def __init__(self, config, physics, P, nlp, server, ctx, start="random", seed=0):
        self.config, self.physics, self.P, self.nlp, self.server = config, physics, P, nlp, server
        self.dt = 1.0 / config["simulation"]["hz"]
        labels = [label(s, k) for s, k in sequences(config["shot_mpc"]["K"])]
        self.geom = geometry(config, labels)
        self.ch = setup_channels(ctx, labels, solves=True)
        self.ch["stats"] = json_channel(ctx, "/live/stats", "holohover.LiveStats", STATS_SCHEMA)

        self.t0_ns = time.time_ns()
        self.clock = 0.0  # simulated time of the session, which sets the log times
        self.start, self.seed, self.rng, self.draw = start, seed, np.random.default_rng(seed), 0
        self.n_episodes = 0
        self.ep = None
        self.window = deque()  # (wall time, clock) over the last second, for the achieved speed

    def ns(self, clock=None):
        return self.t0_ns + round((self.clock if clock is None else clock) * 1e9)

    def episode_ns(self, t):
        """Log time of time t within the current episode."""
        return self.ns(self.t_start + t)

    def event(self, ns, text, level=fm.LogLevel.Info):
        log_event(self.ch, ns, level, text)

    ### Episodes ###
    def new_episode(self, scenario=None, seed=None, reset=False):
        """Start the next episode: the current start again (a new draw if random) or `scenario`.
        reset: requested by /reset, possibly before the current episode is over."""
        if self.ep is not None:
            self._log_outcome(reset)
            self.clock += self.dt  # the new episode's first row must not share the last one's time
        if scenario is not None:
            self.start = scenario
        if seed is not None:
            self.seed, self.rng, self.draw = seed, np.random.default_rng(seed), 0
        if self.start == "random":
            init = sample_initial_state(self.rng, self.config)
            what = f"random draw {self.draw} (seed {self.seed})"
            self.draw += 1
            self.duration = self.config["shot_mpc"]["evaluation"]["duration"]
        else:
            init = initial_state(self.config, self.start)
            what = f"scenario {self.start}"
            self.duration = self.config["simulation"]["duration"]

        self.ep = Episode(self.config, self.physics, self.P, self.nlp, init, verbose=False)
        self.t_start = self.clock
        self.trail = deque([init[12:14]], maxlen=TRAIL_LENGTH)
        self.mode, self.score = None, np.zeros(2, dtype=int)
        self.n_contacts, self.n_plans, self.n_updates = 0, 0, 0
        self.prev_plan = (-np.inf, None)  # (t, label) of the last accepted plan

        ns = self.ns()
        clear_overlays(self.ch, ns)
        log_bodies(self.ch, init, 0.0, ns)
        self.event(ns, f"episode {self.n_episodes}: {what}")
        self.server.broadcast_time(ns)
        self.n_episodes += 1

    def episode_over(self):
        ep = self.ep
        if ep.i_goal is not None:
            return ep.i - 1 >= ep.i_goal + round(RESTART_AFTER_GOAL / self.dt)
        return ep.i * self.dt >= self.duration - 1e-9

    def _log_outcome(self, reset):
        ep = self.ep
        outcome = ep.final_outcome()
        goal = f" at t={ep.t_goal:.2f}s" if ep.t_goal is not None else ""
        cut = f" (reset at t={ep.i * self.dt:.2f}s)" if reset else ""
        self.event(self.ns(), f"outcome: {outcome}{goal}{cut}",
                   fm.LogLevel.Info if outcome == "goal" else fm.LogLevel.Warning)

    ### Steps ###
    def step(self, speed):
        """One control step of the episode, logged; returns its compute time (controller + physics)."""
        t_wall = time.perf_counter()
        row = self.ep.step()
        step_s = time.perf_counter() - t_wall

        t = self.ep.i * self.dt
        self.clock = self.t_start + t
        ns = self.ns()
        self.trail.append(row["states"][12:14])
        log_step(self.ch, self.geom, SimpleNamespace(
            state=row["states"], input=row["inputs"], plan=row["plans"], tf=row["tfs"], mode=row["modes"],
            score=row["scores"], future=row["future"], trail=np.asarray(self.trail),
            cand={key: row["cand_" + key] for key in ("robot", "shot", "score", "valid", "chosen")},
        ), t, ns)
        self._log_events(row, ns)
        self._log_stats(t, ns, speed, step_s)
        self.server.broadcast_time(ns)
        return step_s

    def _log_events(self, row, ns):
        """The events and selector updates of foxglove_viz._events, as they happen."""
        ctrl = self.ep.ctrl
        mode = int(row["modes"])
        if self.mode is not None and mode != self.mode:
            self.event(ns, f"mode {MODE_NAMES[mode]}")
        self.mode = mode
        score = row["scores"]
        for robot in (0, 1):
            if score[robot] > self.score[robot]:
                self.event(ns, f"goal for robot{robot + 1}: {score[0]}:{score[1]}", fm.LogLevel.Warning)
        self.score = score.copy()

        for t_contact in ctrl.contact_times[self.n_contacts:]:
            self.event(self.episode_ns(t_contact), "contact robot1-puck")
        self.n_contacts = len(ctrl.contact_times)
        for t_plan, name, tf in ctrl.plan_history[self.n_plans:]:
            # A plan is re-accepted at every planning tick: report switches and the first plan after a contact
            prev_t, prev_name = self.prev_plan
            if name != prev_name or any(prev_t < c <= t_plan for c in ctrl.contact_times):
                self.event(self.episode_ns(t_plan), f"plan {name}: impact in {tf:.2f}s")
            self.prev_plan = (t_plan, name)
        self.n_plans = len(ctrl.plan_history)
        for log in ctrl.selector.logs[self.n_updates:]:
            self.ch["solves"].log(solve_message(update_json(log)), log_time=self.episode_ns(log.t))
        self.n_updates = len(ctrl.selector.logs)

    def _log_stats(self, t, ns, speed, step_s):
        now = time.perf_counter()
        self.window.append((now, self.clock))
        while now - self.window[0][0] > 1.0:
            self.window.popleft()
        (wall0, clock0), elapsed = self.window[0], now - self.window[0][0]
        achieved = (self.clock - clock0) / elapsed if elapsed > 0.1 else None
        self.ch["stats"].log({"t": t, "speed": speed, "achieved_speed": achieved, "step_ms": 1e3 * step_s},
                             log_time=ns)

    ### Loop ###
    def run(self, controls):
        """Step, log and sleep so that simulated time runs at `speed` times wall time, until interrupted.
        Starts once the app subscribes, so that the first episode is seen from its start."""
        print("Waiting for the Foxglove app to connect...", flush=True)
        while not controls.subscribed.wait(PAUSE_POLL):
            pass
        time.sleep(SUBSCRIBE_GRACE)
        self.new_episode()
        log_static(self.ch, self.config, self.ns())
        anchor = None  # (wall time, clock, speed) that the schedule counts from
        while True:
            if controls.resend_static.is_set():
                controls.resend_static.clear()
                log_static(self.ch, self.config, self.ns())
            if controls.params_changed.is_set():
                controls.params_changed.clear()
                self.server.publish_parameter_values(controls.parameters())
            while not controls.resets.empty():
                self.new_episode(*controls.resets.get(), reset=True)
            v = controls.values()
            if v["paused"]:
                anchor = None
                self.window.clear()
                if controls.take_step():
                    if v["auto_restart"] and self.episode_over():
                        self.new_episode()
                    self.step(v["speed"])
                else:
                    time.sleep(PAUSE_POLL)
                continue
            if v["auto_restart"] and self.episode_over():
                self.new_episode()

            if anchor is None or anchor[2] != v["speed"]:
                anchor = (time.perf_counter(), self.clock, v["speed"])
                self.window.clear()
            self.step(v["speed"])
            delay = anchor[0] + (self.clock - anchor[1]) / anchor[2] - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif -delay > MAX_LAG:
                anchor = None  # held up by a long solve: carry on from here rather than fast-forward


############
### Main ###
############
def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", default="config/config_shot.json")
    parser.add_argument("--scenario", default="random", help="random (default) | stationary | incoming | around")
    parser.add_argument("--seed", type=int, default=0, help="seed of the random starts")
    parser.add_argument("--speed", type=float, default=0.5, help="fraction of real time (default 0.5)")
    parser.add_argument("--no-auto-restart", action="store_true", help="keep playing after a goal")
    parser.add_argument("--host", default="127.0.0.1", help="0.0.0.0 to reach it from outside a container")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-jit", action="store_true")
    parser.add_argument("--no-tracker", action="store_true", help="apply the plan's inputs open loop")
    parser.add_argument("--opponent", choices=["none", "static"], default=None)
    args = parser.parse_args()

    config = load_config(args.config)
    scenarios = list(config["shot_mpc"]["scenarios"])
    if args.scenario not in ("random", *scenarios):
        parser.error(f"--scenario: random or one of {', '.join(scenarios)}")
    if args.no_jit:
        config["shot_mpc"]["jit"] = False
    if args.no_tracker:
        config["shot_mpc"]["tracker"]["enabled"] = False
    if args.opponent:
        config["shot_mpc"]["opponent"] = args.opponent
    physics = physics_from_config(config)
    P = ShotParams.from_config(config)
    nlp = ShotNLP(P)
    # Compile the jitted physics now rather than in the first live step
    Episode(config, physics, P, nlp, initial_state(config), verbose=False).step()

    ctx = foxglove.Context()
    controls = Controls(float(np.clip(args.speed, *SPEED_RANGE)), not args.no_auto_restart, scenarios)
    server = foxglove.start_server(
        name="holohover python-sim", host=args.host, port=args.port, context=ctx,
        capabilities=[Capability.Parameters, Capability.Services, Capability.Time],
        server_listener=controls, supported_encodings=["json"], services=services(controls),
    )
    print(f"Foxglove WebSocket on ws://{args.host}:{server.port}; open {server.app_url()}\n"
          f"and import foxglove/shot_mpc_live_layout.json. Ctrl-C to stop.", flush=True)
    sim = LiveSim(config, physics, P, nlp, server, ctx, start=args.scenario, seed=args.seed)
    try:
        sim.run(controls)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    main()
