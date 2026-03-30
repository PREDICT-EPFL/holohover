"""
Contains definition of the airhockey table on which the robot(s) and puck move.
"""

import time
import os

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter, FFMpegWriter
import matplotlib

matplotlib.use("TkAgg")
import numpy as np
import jax.numpy as jnp
import jax

from rollout import rollout


class TrajecoryAnimation:
    def __init__(
        self,
        states,
        input_sequence,
        path_plans,
        tfs,
        future_trajs,
        modes,
        scores,
        diffusion_trajs,
        diffusion_costs,
        physics,
        config,
    ):
        self.dimension = (physics.table.width, physics.table.height)
        self.goal_width = physics.table.goal_width

        self.robot = physics.robot
        self.puck = physics.puck

        self.robot_state = states[:, :6]
        self.puck_state = states[:, 6:]

        self.input_sequence = input_sequence  # dim (time_steps, horizon, 3)
        self.robot_pp = path_plans[:, :, :6]  # robot path plan (x, y, phi, vx, vy, omega)
        self.input_pp = path_plans[:, :, 6:]  # input path plan (Fx, Fy, tau)
        self.tfs = tfs
        self.ghost_puck_trajs = future_trajs[:, :, 6:9]
        self.robot_initial_state = self.robot_state[0]
        self.puck_initial_state = self.puck_state[0]
        self.modes = modes
        self.scores = scores
        self.diffusion_trajs = diffusion_trajs
        self.diffusion_costs = diffusion_costs

        self.animation_mode = "standard"
        self.dial_time_idx = 0
        self.dial_diff_step = 0
        self.dial_n_steps = int(self.diffusion_trajs.shape[1])
        self.dial_n_candidates = int(self.diffusion_trajs.shape[2])

        self.H = config["simulation"]["H"]
        self.dt = 1 / config["simulation"]["hz"]
        self.horizon_time = self.H * self.dt
        self.duration = config["simulation"]["duration"]
        self.dt_anim = 1 / config["simulation"]["animation_hz"]
        self.fps = config["simulation"]["animation_hz"]
        self.dt_pp = 1 / config["simulation"]["path_planning_hz"]
        self.replan_every = max(1, int(np.round(self.dt_pp / self.dt)))
        self.current_step = 0
        self.max_steps = int(self.duration / self.dt_anim)
        self.n_steps = self.robot_state.shape[0]

        self.is_finished = False
        self.is_zoomed = False

        self.time_array = np.arange(0, self.duration+self.dt, self.dt)
        self.time_arrays_pp = np.zeros((self.tfs.shape[0], self.H))
        for i, tf in enumerate(self.tfs):
            if i == 0:
                start_idx = 0
            else:
                start_idx = ((i - 1) // self.replan_every) * self.replan_every + 1
            self.time_arrays_pp[i, :] = np.linspace(
                start_idx * self.dt, start_idx * self.dt + tf, self.H
            )

        self.robot_vx = self.robot_state[:, 3]
        self.robot_vy = self.robot_state[:, 4]
        self.puck_vx = self.puck_state[:, 3]
        self.puck_vy = self.puck_state[:, 4]
        self.fx = input_sequence[:, 0]
        self.fy = input_sequence[:, 1]
        self.tau = input_sequence[:, 2]

        self.fig, self.ax = None, None
        self.robot_patch = None
        self.robot_orientation_dot = None
        self.robot_ol_pred_patch = None
        self.robot_pp_patch = None
        self.puck_patch = None
        self.puck_trail_patch = None
        self.ghost_puck_traj_patch = None
        self.ghost_puck_patch = None
        self.slider = None

        self.dial_info_text = None
        self.dial_final_pp_patch = None
        self.dial_ghost_puck_patch = None
        self.dial_candidate_lines = []
        self.dial_candidate_dots = []

        # Compute OL Robot position prediction
        self.ol = self._compute_ol(states, input_sequence, physics)  # dim TxHx12
        self.init_ol = self.ol[0]

        # Puck trail (Ai magic)
        trail_length = 60
        step_indices = np.arange(self.n_steps)
        start_indices = np.maximum(0, step_indices[:, None] - trail_length + np.arange(trail_length))
        start_indices = np.minimum(start_indices, step_indices[:, None])
        self.puck_trail_x = self.puck_state[start_indices, 0]
        self.puck_trail_y = self.puck_state[start_indices, 1]

    def _compute_ol(self, states, input_sequence, physics):
        dt = self.dt

        batched_rollout = jax.vmap(rollout, in_axes=(0, 0, None, None))

        trajectories, _ = batched_rollout(states, input_sequence, physics, dt)

        return trajectories[:, :, :3]  # only robot position

    def _setup_table_axis(self, key="table", store_defaults=False):
        """Configure table axis, border, goals and center line."""
        axis = self.ax[key]
        width, height = self.dimension
        goal_y = (height - self.goal_width) / 2

        axis.set_aspect("equal")
        axis.set_xlim(0, width)
        axis.set_ylim(0, height)
        axis.set_autoscale_on(False)
        axis.axis("off")

        if store_defaults:
            self.default_xlim = axis.get_xlim()
            self.default_ylim = axis.get_ylim()

        table_border = plt.Rectangle(
            (0, 0), width, height, linewidth=3, edgecolor="black", facecolor="#f7f7f7"
        )
        axis.add_patch(table_border)

        self.left_goal_patch = plt.Rectangle(
            (0, goal_y),
            width / 30,
            self.goal_width,
            color="red",
            alpha=0.6,
        )
        axis.add_patch(self.left_goal_patch)

        self.right_goal_patch = plt.Rectangle(
            (width - width / 30, goal_y),
            width / 30,
            self.goal_width,
            color="blue",
            alpha=0.6,
        )
        axis.add_patch(self.right_goal_patch)

        axis.plot(
            [width / 2, width / 2],
            [0, height],
            color="gray",
            linestyle="--",
            linewidth=2,
        )

    def _set_slider_value(self, val):
        """Set slider value without triggering callbacks recursively."""
        self.slider.eventson = False
        self.slider.set_val(val)
        self.slider.eventson = True

    def _next_dial_time_idx(self, idx):
        """Return next valid DIAL time index in sequence 0, 1, 1 + k*replan_every."""
        if idx < 0:
            return 0
        if idx == 0:
            return min(self.n_steps - 1, 1)

        step = self.replan_every
        k = ((idx - 1) // step) + 1
        return min(self.n_steps - 1, 1 + k * step)

    def _prev_dial_time_idx(self, idx):
        """Return previous valid DIAL time index in sequence 0, 1, 1 + k*replan_every."""
        if idx <= 0:
            return 0
        if idx <= 1:
            return 0

        step = self.replan_every
        k = (idx - 1) // step
        prev_idx = 1 + (k - 1) * step
        return max(1, prev_idx)

    def setup_display(self, show=True):
        """Initialize the matplotlib figure and axes for display."""
        if show:
            plt.ion()
        else:
            plt.ioff()

        self.fig, self.ax = plt.subplot_mosaic(
            [
                ["table", "table", "table", "rvel", "pvel"],
                ["table", "table", "table", "fx", "fy"],
                ["slider", "slider", "slider", "replay", "zoom"],
            ],
            figsize=(14, 7),
            gridspec_kw={"hspace": 0.3, "wspace": 0.4, "height_ratios": [3, 3, 0.5]},
        )
        ##################
        ### Table axis ###
        ##################
        self._setup_table_axis(key="table", store_defaults=True)

        # Info text above the table
        self.info_text = self.ax["table"].text(
            1,
            1.02,
            "",
            transform=self.ax["table"].transAxes,
            ha="right",
            va="bottom",
            fontsize=12,
            fontweight="bold",
            animated=True,
        )

        # Display Robot position path plan
        init_pos_pp = self.robot_pp[0, :, :2]
        self.robot_pp_patch = plt.Line2D(
            init_pos_pp[:, 0],
            init_pos_pp[:, 1],
            linewidth=1.5,
            marker=".",
            markersize=4,
            color="red",
            alpha=0.9,
            animated=True,
        )
        self.ax["table"].add_line(self.robot_pp_patch)

        # Display OL robot position predition
        self.robot_ol_pred_patch = plt.Line2D(
            self.init_ol[:, 0],
            self.init_ol[:, 1],
            linewidth=2,
            marker="o",
            markersize=3,
            color="grey",
            alpha=0.5,
            animated=True,
        ) # Might become useful again later if well use MPC to track Path Plan
        # self.ax["table"].add_line(self.robot_ol_pred_patch)

        # Create robot circle (after OL s.t. Robot stays in foreground)
        rx, ry, rphi = self.robot_initial_state[:3]
        self.robot_patch = plt.Circle(
            (rx, ry),
            self.robot.radius,
            color="black",
            alpha=0.6,
            animated=True,
        )
        self.ax["table"].add_patch(self.robot_patch)

        # Add orientation indicator dot
        angle = rphi  # phi
        # Position on circumference: radius * direction
        dot_pos = (rx, ry) + 0.6 * self.robot.radius * np.array(
            [np.cos(angle), np.sin(angle)]
        )
        self.robot_orientation_dot = plt.Circle(
            dot_pos, 0.08 * self.robot.radius, color="yellow", animated=True
        )
        self.ax["table"].add_patch(self.robot_orientation_dot)

        # Create puck circle
        px, py, _ = self.puck_initial_state[:3]
        self.puck_patch = plt.Circle(
            (px, py),
            self.puck.radius,
            color="green",
            animated=True,
        )
        self.ax["table"].add_patch(self.puck_patch)

        # Puck trail
        self.puck_trail_patch = plt.Line2D(
            self.puck_trail_x[0],
            self.puck_trail_y[0],
            linewidth=1,
            color="green", 
            # marker="o",
            # markersize=2,
            alpha=0.55,
            animated=True,
        )
        self.ax["table"].add_line(self.puck_trail_patch)

        # Puck predicted future positions
        init_ghost_traj = self.ghost_puck_trajs[0, :, :2]  # only xy, discard phi
        self.ghost_puck_traj_patch = plt.Line2D(
            init_ghost_traj[:, 0],
            init_ghost_traj[:, 1],
            linewidth=0.1,
            marker="o",
            markersize=3,
            color="grey",
            alpha=0.4,
            animated=True,
        )
        self.ax["table"].add_line(self.ghost_puck_traj_patch)
        # Puck predicted end position
        self.ghost_puck_patch = plt.Circle(
            init_ghost_traj[-1, :],
            self.puck.radius,
            color="green",
            alpha=0.4,
            animated=True,
        )
        self.ax["table"].add_patch(self.ghost_puck_patch)

        ##############
        ### Slider ###
        ##############
        self.ax["slider"].set_axis_off()
        self.ax["slider"].set_visible(False)
        self.slider = matplotlib.widgets.Slider(
            self.ax["slider"],
            "Time",
            0,
            self.duration,
            valinit=self.duration,
            valstep=self.dt_anim,
        )

        self.slider.on_changed(self.update_slider)
        self.slider.set_active(False)

        ##############
        ### Button ###
        ##############
        self.ax["replay"].set_axis_off()
        self.replay_btn = matplotlib.widgets.Button(self.ax["replay"], "↺ Replay")
        self.replay_btn.on_clicked(self.restart_animation)
        self.replay_btn.label.set_color("black")
        self.replay_btn.label.set_weight("bold")
        self.replay_btn.label.set_fontsize(14)

        self.ax["zoom"].set_axis_off()
        self.zoom_btn = matplotlib.widgets.Button(self.ax["zoom"], "Zoom")
        self.zoom_btn.on_clicked(self.zoom_animation)
        self.zoom_btn.label.set_color("blue")
        self.zoom_btn.label.set_weight("bold")
        self.zoom_btn.label.set_fontsize(14)

        ###########################
        ### Robot Velocity plot ###
        ###########################
        self.ax["rvel"].set_title(
            "Robot Velocity", fontdict={"fontsize": 10, "fontweight": "bold"}
        )
        self.ax["rvel"].set_xlabel("time (s)")
        # self.ax["rvel"].set_ylabel("Velocity (m/s)")
        self.ax["rvel"].set_xlim(0, self.duration)
        vmax = max(np.max(np.abs(self.robot_vx)), np.max(np.abs(self.robot_vy)))
        self.ax["rvel"].set_ylim(-1.2 * vmax, 1.2 * vmax)

        # Acutal plot
        cmap = plt.get_cmap("inferno")
        self.ax["rvel"].plot(
            self.time_array, self.robot_vx, label="vx", color=cmap(0.3)
        )
        self.ax["rvel"].plot(
            self.time_array, self.robot_vy, label="vy", color=cmap(0.6)
        )

        # Time marker
        self.rvel_time_marker = self.ax["rvel"].axvline(
            0, animated=True, color="black", alpha=0.5
        )
        self.ax["rvel"].legend(loc="upper left")

        ##########################
        ### Puck Velocity plot ###
        ##########################
        self.ax["pvel"].set_title(
            "Puck Velocity", fontdict={"fontsize": 10, "fontweight": "bold"}
        )
        self.ax["pvel"].set_xlabel("time (s)")
        # self.ax["pvel"].set_ylabel("Velocity (m/s)")
        self.ax["pvel"].set_xlim(0, self.duration)
        vmax = max(np.max(np.abs(self.puck_vx)), np.max(np.abs(self.puck_vy)))
        self.ax["pvel"].set_ylim(-1.2 * vmax, 1.2 * vmax)

        # Acutal plot
        cmap = plt.get_cmap("inferno")
        self.ax["pvel"].plot(self.time_array, self.puck_vx, label="vx", color=cmap(0.3))
        self.ax["pvel"].plot(self.time_array, self.puck_vy, label="vy", color=cmap(0.6))

        # Time marker
        self.pvel_time_marker = self.ax["pvel"].axvline(
            0, animated=True, color="black", alpha=0.5
        )
        self.ax["pvel"].legend(loc="upper left")

        ##########################
        ### Control input plot ###
        ##########################
        self.ax["fx"].set_title("Fx", fontdict={"fontsize": 10, "fontweight": "bold"})
        self.ax["fx"].set_xlabel("time (s)")
        self.ax["fy"].set_title("Fy", fontdict={"fontsize": 10, "fontweight": "bold"})
        self.ax["fy"].set_xlabel("time (s)")
        # ax_tau = self.ax["u"].twinx()

        # self.ax["fx"].set_ylabel("Force (N)")
        # ax_tau.set_ylabel("Torque (Nm)")

        self.ax["fx"].set_xlim(0, self.duration)
        self.ax["fy"].set_xlim(0, self.duration)
        fx_max = np.max(np.abs(self.fx))
        fy_max = np.max(np.abs(self.fy))
        self.ax["fx"].set_ylim(-1.2 * fx_max, 1.2 * fx_max)
        self.ax["fy"].set_ylim(-1.2 * fy_max, 1.2 * fy_max)
        # taumax = np.max(np.abs(self.tau))
        # ax_tau.set_ylim(-1.2 * taumax, 1.2 * taumax)

        # Acutal plot
        cmap = plt.get_cmap("inferno")
        self.ax["fx"].plot(self.time_array, self.fx, label="Fx", color=cmap(0.3))
        self.ax["fy"].plot(self.time_array, self.fy, label="Fy", color=cmap(0.3))
        fx_planned = self.input_pp[0, :, 0]
        fy_planned = self.input_pp[0, :, 1]
        self.fx_pp_plot = self.ax["fx"].plot(
            self.time_arrays_pp[0],
            fx_planned,
            label="Planned Fx",
            color=cmap(0.9),
            animated=True,
        )[0]
        self.fy_pp_plot = self.ax["fy"].plot(
            self.time_arrays_pp[0],
            fy_planned,
            label="Planned Fy",
            color=cmap(0.9),
            animated=True,
        )[0]
        # ax_tau.plot(self.time_array, self.tau, label="Tau", color=cmap(0.9))

        # Time marker
        self.fx_time_marker = self.ax["fx"].axvline(
            0, animated=True, color="black", alpha=0.5
        )
        self.fy_time_marker = self.ax["fy"].axvline(
            0, animated=True, color="black", alpha=0.5
        )

        self.ax["fx"].legend(loc="upper left")
        self.ax["fy"].legend(loc="upper left")
        # ax_tau.legend(loc=0)

        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)

        # Connect keyboard event handler
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

        if show:
            plt.show(block=False)

    def set_blitting_enabled(self, enabled):
        """Enable or disable animated artists used by blitting."""
        artists = [
            self.robot_pp_patch,
            self.robot_patch,
            self.robot_orientation_dot,
            self.puck_patch,
            self.puck_trail_patch,
            self.ghost_puck_traj_patch,
            self.ghost_puck_patch,
            self.fx_pp_plot,
            self.fy_pp_plot,
            self.rvel_time_marker,
            self.pvel_time_marker,
            self.fx_time_marker,
            self.fy_time_marker,
            self.info_text,
        ]
        for artist in artists:
            if artist is not None:
                artist.set_animated(enabled)

    def setup_dial_display(self, show=True):
        """Initialize a minimal keyboard-only display for DIAL trajectory debugging."""
        if show:
            plt.ion()
        else:
            plt.ioff()

        self.fig, self.ax = plt.subplot_mosaic(
            [["table"],
             ["slider"]],
            figsize=(10, 7),
            gridspec_kw={"hspace": 0.25, "height_ratios": [8, 1]},
        )
        self._setup_table_axis(key="table", store_defaults=False)


        rx, ry, rphi = self.robot_initial_state[:3]
        self.robot_patch = plt.Circle((rx, ry), self.robot.radius, color="black", alpha=0.6)
        self.ax["table"].add_patch(self.robot_patch)

        dot_pos = (rx, ry) + 0.6 * self.robot.radius * np.array([np.cos(rphi), np.sin(rphi)])
        self.robot_orientation_dot = plt.Circle(
            dot_pos, 0.08 * self.robot.radius, color="yellow"
        )
        self.ax["table"].add_patch(self.robot_orientation_dot)

        px, py, _ = self.puck_initial_state[:3]
        self.puck_patch = plt.Circle((px, py), self.puck.radius, color="green", alpha=0.6)
        self.ax["table"].add_patch(self.puck_patch)


        self.dial_info_text = self.ax["table"].text(
            0.01,
            1.02,
            "",
            transform=self.ax["table"].transAxes,
            ha="left",
            va="bottom",
            fontsize=11,
            fontweight="bold",
        )

        self.dial_candidate_lines = []
        self.dial_candidate_dots = []
        cmap = plt.get_cmap("tab10")
        for k in range(self.dial_n_candidates):
            color = cmap(k % 10)
            line = plt.Line2D(
                [],
                [],
                linewidth=2.0,
                color=color,
                alpha=0.7,
                marker="o",
                markersize=2.5,
                markevery=max(1, self.H // 5),
            )
            self.ax["table"].add_line(line)
            self.dial_candidate_lines.append(line)

            dot = plt.Circle((0, 0), 0.1 * self.robot.radius, color=color, alpha=0.7)
            self.ax["table"].add_patch(dot)
            self.dial_candidate_dots.append(dot)


        init_ghost_xy = self.ghost_puck_trajs[0, 0, :2]
        self.dial_ghost_puck_patch = plt.Circle(
            init_ghost_xy,
            self.puck.radius,
            color="lime",
            alpha=0.35,
            label="Predicted puck",
        )
        self.ax["table"].add_patch(self.dial_ghost_puck_patch)

        init_pos_pp = self.robot_pp[0, :, :2]
        self.dial_final_pp_patch = plt.Line2D(
            init_pos_pp[:, 0],
            init_pos_pp[:, 1],
            linewidth=1.0,
            color="black",
            label="Final path plan",
        )
        self.ax["table"].add_line(self.dial_final_pp_patch)
        self.ax["table"].legend(loc="best", fontsize=8)


        self.ax["slider"].set_axis_off()
        self.slider = matplotlib.widgets.Slider(
            self.ax["slider"],
            "Time",
            0,
            self.duration,
            valinit=0,
            valstep=self.dt,
        )
        self.slider.set_active(False)

        self.fig.canvas.mpl_connect("key_press_event", self.on_key_press)

        self.fig.canvas.draw()
        if show:
            plt.show(block=False)

    def on_key_press(self, event):
        """Dispatch keyboard controls based on the selected animation mode."""
        if self.animation_mode == "dial":
            self.on_key_press_dial(event)
            return
        self.on_key_press_standard(event)

    def on_key_press_standard(self, event):
        """Handle keyboard input for the standard animation mode."""
        if event.key not in ['left', 'right', 'backspace', ' ']:
            return
        
        if event.key == 'left':
            # Move slider left (decrease time)
            if self.is_finished:
                new_step = max(0, self.current_step - 1)
                self.current_step = new_step
                new_val = new_step * self.duration / self.max_steps
                self._set_slider_value(new_val)
        
        elif event.key == 'right':
            # Move slider right (increase time)
            if self.is_finished:
                new_step = min(self.max_steps, self.current_step + 1)
                self.current_step = new_step
                new_val = new_step * self.duration / self.max_steps
                self._set_slider_value(new_val)
        
        elif event.key == 'backspace':
            # Replay animation
            self.restart_animation(None)
        
        elif event.key == ' ':
            # Pause/Resume animation
            if self.is_finished:
                # Resume animation
                self.is_finished = False
                self.ax["slider"].set_visible(False)
                self.slider.set_active(False)

                self.set_blitting_enabled(True)
                self.fig.canvas.draw()
                self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
            else:
                val = self.current_step * self.duration / self.max_steps
                self._set_slider_value(val)
                # Pause animation
                self.is_finished = True

                self.set_blitting_enabled(False)
                # Force a full redraw so the slider appears
                self.fig.canvas.draw()

                self.slider.set_active(True)
                self.ax["slider"].set_visible(True)

    def on_key_press_dial(self, event):
        """Handle keyboard-only navigation for DIAL debug mode."""
        if event.key not in ["left", "right", "up", "down"]:
            return

        if event.key == "down":
            self.dial_diff_step = max(0, self.dial_diff_step - 1)
        elif event.key == "up":
            self.dial_diff_step = min(self.dial_n_steps - 1, self.dial_diff_step + 1)

        elif event.key == "left":
            self.dial_time_idx = self._prev_dial_time_idx(self.dial_time_idx)
        elif event.key == "right":
            self.dial_time_idx = self._next_dial_time_idx(self.dial_time_idx)

        self.update_dial_display()

    def update_display(self, idx=None, zoom=True):
        """Update the visual positions of robots and puck."""
        canvas = self.fig.canvas
        canvas.restore_region(self.background)

        real_time_idx = int(idx * self.dt_anim / self.dt)
        sim_time = real_time_idx * self.dt

        # Example placeholders; replace with your real values
        current_tf = self.tfs[real_time_idx]
        current_step = real_time_idx
        current_mode = self.modes[real_time_idx]
        current_score = self.scores[real_time_idx]

        # Update info text
        tf_text = f"{current_tf:.2f}s" if current_mode == 1 else "N/A"
        mode_text = "ATT" if current_mode == 1 else "DEF"
        score_text = f"{current_score[1]} : {current_score[0]}"
        self.info_text.set_text(
            f"tf = {tf_text}\nt = {current_step*self.dt:.2f}s\nmode: {mode_text}\n{score_text}\nF: {self.fx[real_time_idx]:.3f}, {self.fy[real_time_idx]:.3f}"
        )
        self.ax["table"].draw_artist(self.info_text)

        # Update robot
        # Path Plan
        self.robot_pp_patch.set_data(
            self.robot_pp[real_time_idx, :, 0], self.robot_pp[real_time_idx, :, 1]
        )
        self.ax["table"].draw_artist(self.robot_pp_patch)
        # OL prediction
        # self.robot_ol_pred_patch.set_data(self.ol[real_time_idx, :, 0], self.ol[real_time_idx, :, 1])
        # self.ax["table"].draw_artist(self.robot_ol_pred_patch)
        # Robot circle
        pos = self.robot_state[real_time_idx, :3]
        self.robot_patch.center = pos[:2]
        self.ax["table"].draw_artist(self.robot_patch)
        # Orientation dot
        phi = pos[2]
        dot_pos = pos[:2] + 0.6 * self.robot.radius * np.array(
            [np.cos(phi), np.sin(phi)]
        )
        self.robot_orientation_dot.center = dot_pos
        self.ax["table"].draw_artist(self.robot_orientation_dot)

        # Update puck
        pos = self.puck_state[real_time_idx, :3]
        self.puck_patch.center = pos[:2]

        self.puck_trail_patch.set_data(self.puck_trail_x[real_time_idx],
                                       self.puck_trail_y[real_time_idx])


        self.ax["table"].draw_artist(self.puck_trail_patch)
        self.ax["table"].draw_artist(self.puck_patch)

        self.ghost_puck_traj_patch.set_data(
            self.ghost_puck_trajs[real_time_idx, :, 0],
            self.ghost_puck_trajs[real_time_idx, :, 1],
        )
        self.ax["table"].draw_artist(self.ghost_puck_traj_patch)
        self.ghost_puck_patch.center = self.ghost_puck_trajs[real_time_idx, -1, :2]
        self.ax["table"].draw_artist(self.ghost_puck_patch)

        # Update plots
        fx_planned = self.input_pp[real_time_idx, :, 0]
        fy_planned = self.input_pp[real_time_idx, :, 1]

        if current_mode == 1:  # if on Attack
            self.fx_pp_plot.set_data(self.time_arrays_pp[real_time_idx], fx_planned)
            self.fy_pp_plot.set_data(self.time_arrays_pp[real_time_idx], fy_planned)
            self.ax["fx"].draw_artist(self.fx_pp_plot)
            self.ax["fy"].draw_artist(self.fy_pp_plot)

        self.rvel_time_marker.set_xdata([sim_time])
        self.ax["rvel"].draw_artist(self.rvel_time_marker)

        self.pvel_time_marker.set_xdata([sim_time])
        self.ax["pvel"].draw_artist(self.pvel_time_marker)

        self.fx_time_marker.set_xdata([sim_time])
        self.fy_time_marker.set_xdata([sim_time])
        self.ax["fx"].draw_artist(self.fx_time_marker)
        self.ax["fy"].draw_artist(self.fy_time_marker)

        # Zoom in
        if zoom:
            # Table Zoom
            rx, ry = self.robot_state[real_time_idx, :2]
            zoom_size = self.robot.radius * 4

            x_min = rx - zoom_size
            x_max = rx + zoom_size
            y_min = ry - zoom_size
            y_max = ry + zoom_size
            width, height = self.dimension

            if x_min < 0:
                x_max -= x_min
                x_min = 0
            elif x_max > width:
                x_min -= x_max - width

            if y_min < 0:
                y_max -= y_min
                y_min = 0
            elif y_max > height:
                y_min -= y_max - height

            self.ax["table"].set_xlim(x_min, x_max)
            self.ax["table"].set_ylim(y_min, y_max)

            # Plot Zoom
            t_min = max(0, sim_time - self.horizon_time)
            t_max = min(self.duration, sim_time + 4 * self.horizon_time)

            self.ax["rvel"].set_xlim(t_min, t_max)
            self.ax["pvel"].set_xlim(t_min, t_max)
            self.ax["fx"].set_xlim(t_min, t_max)
            self.ax["fy"].set_xlim(t_min, t_max)

        canvas.blit(self.fig.bbox)
        canvas.flush_events()

    def update_dial_display(self):
        """Update DIAL debug visualization for selected time index and diffusion step."""
        self.dial_time_idx = int(np.clip(self.dial_time_idx, 0, self.n_steps - 1))
        self.dial_diff_step = int(np.clip(self.dial_diff_step, 0, self.dial_n_steps - 1))

        pos_robot = np.asarray(self.robot_state[self.dial_time_idx, :3])
        self.robot_patch.center = pos_robot[:2]
        phi = pos_robot[2]
        dot_pos = pos_robot[:2] + 0.6 * self.robot.radius * np.array([np.cos(phi), np.sin(phi)])
        self.robot_orientation_dot.center = dot_pos

        pos_puck = np.asarray(self.puck_state[self.dial_time_idx, :3])
        self.puck_patch.center = pos_puck[:2]
        self.dial_ghost_puck_patch.center = self.ghost_puck_trajs[
            self.dial_time_idx, 0, :2
        ]

        candidate_trajs = np.asarray(
            self.diffusion_trajs[self.dial_time_idx, self.dial_diff_step]
        )
        candidate_costs = np.asarray(
            self.diffusion_costs[self.dial_time_idx, self.dial_diff_step]
        )

        normalized_weights = candidate_costs - np.min(candidate_costs)
        normalized_weights /= np.max(normalized_weights)


        for k, line in enumerate(self.dial_candidate_lines):
            traj_k = candidate_trajs[k]
            wk = float(normalized_weights[k])
            line.set_data(traj_k[:, 0], traj_k[:, 1])
            line.set_linewidth(0.5 + 2.0 * wk)
            line.set_alpha(0.5)

            end_xy = traj_k[-1]
            self.dial_candidate_dots[k].center = end_xy
            self.dial_candidate_dots[k].set_radius(0.05 * self.robot.radius + 0.25 * self.robot.radius * wk)
            self.dial_candidate_dots[k].set_alpha(0.5)

        # Initial and final path plan for reference
        self.dial_final_pp_patch.set_data(
            self.robot_pp[self.dial_time_idx, :, 0],
            self.robot_pp[self.dial_time_idx, :, 1],
        )

        sim_time = self.dial_time_idx * self.dt
        self._set_slider_value(sim_time)

        self.dial_info_text.set_text(
            f"DIAL debug\nt = {sim_time:.2f}s ({self.dial_time_idx + 1}/{self.n_steps})\n"
            f"diff step = {self.dial_diff_step + 1}/{self.dial_n_steps}"
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def update_slider(self, val):
        if not self.is_finished:
            return
        self._set_slider_value(val)

        idx = int(val / self.duration * self.max_steps)
        self.current_step = idx
        self.update_display(idx=idx, zoom=self.is_zoomed)

    def restart_animation(self, event):
        """Reset everything to start the animation over."""
        self.current_step = 0
        self.is_finished = False
        self.is_zoomed = False

        self.ax["table"].set_xlim(self.default_xlim)
        self.ax["table"].set_ylim(self.default_ylim)

        self.ax["rvel"].set_xlim(0, self.duration)
        self.ax["pvel"].set_xlim(0, self.duration)
        self.ax["fx"].set_xlim(0, self.duration)
        self.ax["fy"].set_xlim(0, self.duration)

        # Hide UI elements again
        self.ax["slider"].set_visible(False)
        self.slider.set_active(False)

        self.set_blitting_enabled(True)

        # Redraw the background/static elements
        self.fig.canvas.draw()
        self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)

    def zoom_animation(self, event):
        self.is_zoomed = not self.is_zoomed

    def play(self, mode="standard"):
        """Display the animation in standard mode or DIAL debug mode."""
        if mode == "dial":
            self.play_dial_debug()
            return

        self.animation_mode = "standard"
        self.setup_display()

        t1 = time.time()
        while plt.fignum_exists(self.fig.number):
            if not self.is_finished:
                self.current_step = min(self.current_step + 1, self.max_steps)

            if not self.is_finished and self.current_step == self.max_steps:
                self.is_finished = True
                self.slider.set_active(True)
                self.ax["slider"].set_visible(True)

                self.set_blitting_enabled(False)

                # Force a full redraw so the slider and objects appear together
                self.fig.canvas.draw()

                self.current_step = self.max_steps  # stay on last frame
                new_val = self.current_step * self.duration / self.max_steps
                self._set_slider_value(new_val)

            self.update_display(idx=self.current_step, zoom=self.is_zoomed)

            t2 = time.time()
            elapsed = t2 - t1
            sleep_time = max(0, self.dt_anim - elapsed)
            time.sleep(sleep_time)
            t1 = time.time()

    def play_dial_debug(self):
        """Display DIAL candidate trajectories with keyboard-only navigation."""
        self.animation_mode = "dial"
        self.dial_time_idx = 0
        self.dial_diff_step = 0

        self.setup_dial_display()
        self.update_dial_display()

        while plt.fignum_exists(self.fig.number):
            plt.pause(0.05)


    def export_animation(self, filename):
        """
        Export the animation as a GIF.
        """
        # Create a new figure for export (without interactive elements)
        self.setup_display(show=False)

        # Hide interactive UI elements before render
        self.ax["slider"].set_visible(False)
        self.ax["replay"].set_visible(False)
        self.ax["zoom"].set_visible(False)

        # Build frame sequence with skipping
        frame_indices = range(0, self.max_steps, 3)

        # Create animation function for FuncAnimation
        def animate(frame):
            self.update_display(
                idx=frame, zoom=False
            )  # Disable zoom for consistent export
            return []

        # Create FuncAnimation
        anim = animation.FuncAnimation(
            self.fig,
            animate,
            frames=frame_indices,
            interval=1000 / self.fps,
            blit=False,
        )

        # Save animation
        os.makedirs("gifs", exist_ok=True)
        try:
            writer = PillowWriter(fps=self.fps / 3)
            anim.save(f"gifs/{filename}.gif", writer=writer)
            print(f"Animation saved as {filename}.gif")
        except Exception as e:
            print(f"Failed to save GIF: {e}")
            print("Make sure pillow is installed")
