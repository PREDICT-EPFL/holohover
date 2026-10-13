import jax
import jax.numpy as jnp
from flax import struct
import scipy


# Parameters
@struct.dataclass
class Table:
    width: float
    height: float
    goal_width: float
    friction: float

    @property
    def left_goal(self):
        return jnp.array([0, 0.5 * self.height])

    @property
    def right_goal(self):
        return jnp.array([self.width, 0.5 * self.height])


@struct.dataclass
class Entity:
    radius: float
    mass: float
    inertia: float
    u_limits: jnp.array

    @property
    def A(self):
        return jnp.block([[jnp.zeros((3, 3)), jnp.eye(3)], [jnp.zeros((3, 6))]])

    @property
    def B(self):
        M = jnp.array([1 / self.mass, 1 / self.mass, 1 / self.inertia])
        return jnp.block([[jnp.zeros((3, 3))], [jnp.diag(M)]])


@struct.dataclass
class PhysicsParams:
    table: Table
    robot: Entity
    puck: Entity

    def system(self, dt):
        # Match the simulator: position advances with the current velocity,
        # then velocity is damped and actuated.
        drag = 1.0 - self.table.friction * dt
        Ad = jnp.block([[jnp.eye(3), dt * jnp.eye(3)], [jnp.zeros((3, 3)), drag * jnp.eye(3)]])
        M = jnp.diag(jnp.array([1 / self.robot.mass, 1 / self.robot.mass, 1 / self.robot.inertia]))
        Bd = jnp.block([[0.5 * dt**2 * M], [dt * M]])

        return Ad, Bd

    def check_puck_heading_goal(self, robot_id, puck_pos, puck_vel):
        """Check if puck is heading towards goal (even with wall bounces)"""
        x, y = puck_pos[0], puck_pos[1]
        vx, vy = puck_vel[0], puck_vel[1]

        goal_y_min = 0.5 * self.table.height - 0.5 * self.table.goal_width
        goal_y_max = 0.5 * self.table.height + 0.5 * self.table.goal_width

        # lin function y = ax + b
        a = vy / (vx + 1e-8)
        b = y - a * x

        # When do we hit left wall?
        x_hit = jnp.where(robot_id==1, self.puck.radius, self.table.width - self.puck.radius)
        y_hit = a * x_hit + b

        # View wall bounces as transfer to virtual table next to real table
        y_hit_debounced = y_hit % (self.table.height - 2 * self.puck.radius)

        is_heading_to_goal = (y_hit_debounced >= goal_y_min)& (y_hit_debounced <= goal_y_max)
        is_heading_to_goal = jnp.where(robot_id==1,
                                       is_heading_to_goal & (vx < 0),
                                       is_heading_to_goal & (vx > 0))

        return is_heading_to_goal


def wall_collision(pos, vel, radius, table: Table):
    x, y = pos[0], pos[1]
    vx, vy = vel[0], vel[1]

    # detect goal
    goal_y_min = 0.5 * table.height - 0.5 * table.goal_width
    goal_y_max = 0.5 * table.height + 0.5 * table.goal_width
    in_goal_y = (y >= goal_y_min) & (y <= goal_y_max)
    scored_left = (x - radius < 0.0) & in_goal_y
    scored_right = (x + radius > table.width) & in_goal_y

    # 0: hasnt scored, 1: scored on left goal, 2: scored on right goal
    has_scored = jnp.where(scored_left, 1, jnp.where(scored_right, 2, 0))
    hit_y_left = jnp.where((x - radius <= 0.0), y, -1.0)
    hit_y_right = jnp.where((x + radius >= table.width), y, -1.0)

    # detect wall bounces:
    wall_bounces = jnp.zeros(6)  # [TopLeft, T, TR, BottomRight, B, BL]
    z6 = jnp.zeros(6)

    cond_top = y >= goal_y_max
    cond_bottom = y <= goal_y_min

    # X lower
    cond = x - radius < 0.0
    x = jnp.where(cond, 2 * (radius) - x, x)
    vx = jnp.where(cond, -vx, vx)
    wall_bounces += jnp.where(cond & cond_top, jnp.array([1, 0, 0, 0, 0, 0]), z6)
    wall_bounces += jnp.where(cond & cond_bottom, jnp.array([0, 0, 0, 0, 0, 1]), z6)

    # X upper
    cond = x + radius > table.width
    x = jnp.where(cond, 2 * (table.width - radius) - x, x)
    vx = jnp.where(cond, -vx, vx)
    wall_bounces += jnp.where(cond & cond_top, jnp.array([0, 0, 1, 0, 0, 0]), z6)
    wall_bounces += jnp.where(cond & cond_bottom, jnp.array([0, 0, 0, 1, 0, 0]), z6)

    # Y lower
    cond = (y - radius < 0.0) & (-100.0 < y - radius) # -100 just hack for puck resets (where we set y to -200 to make it invisible)
    y = jnp.where(cond, 2 * (radius) - y, y)
    vy = jnp.where(cond, -vy, vy)
    wall_bounces += jnp.where(cond, jnp.array([0, 0, 0, 0, 1, 0]), z6)

    # Y upper
    cond = y + radius > table.height
    y = jnp.where(cond, 2 * (table.height - radius) - y, y)
    vy = jnp.where(cond, -vy, vy)
    wall_bounces += jnp.where(cond, jnp.array([0, 1, 0, 0, 0, 0]), z6)

    pos = jnp.array([x, y, pos[2]])
    vel = jnp.array([vx, vy, vel[2]])

    return pos, vel, has_scored, hit_y_left, hit_y_right, wall_bounces


def robot_puck_collision(r_pos, r_vel, p_pos, p_vel, r_params, p_params):

    delta = r_pos - p_pos
    dist = jnp.linalg.norm(delta)
    min_dist = r_params.radius + p_params.radius

    def collide(_):

        m1, m2 = r_params.mass, p_params.mass

        v1_new = (
            r_vel
            - (2 * m2 / (m1 + m2)) * (jnp.dot(r_vel - p_vel, delta) / (dist**2)) * delta
        )

        v2_new = p_vel - (2 * m1 / (m1 + m2)) * (
            jnp.dot(p_vel - r_vel, -delta) / (dist**2)
        ) * (-delta)

        n = delta / dist
        overlap = min_dist - dist
        correction = 0.5 * overlap * n

        r_pos_new = r_pos + correction
        p_pos_new = p_pos - correction

        return r_pos_new, v1_new, p_pos_new, v2_new, 1

    def no_collision(_):
        return r_pos, r_vel, p_pos, p_vel, 0

    return jax.lax.cond(
        (dist < min_dist) & (dist > 1e-6), collide, no_collision, operand=None
    )


def integrate_body(state, force, entity, table, dt):

    pos = state[:3]
    vel = state[3:]
    Fx, Fy, tau = force
    acc = jnp.array([Fx / entity.mass, Fy / entity.mass, tau / entity.inertia])

    pos = pos + vel * dt + 0.5 * acc * dt**2
    vel = vel * (1 - table.friction * dt) + acc * dt

    # vel = vel.at[0].add(Fx / entity.mass * dt)
    # vel = vel.at[1].add(Fy / entity.mass * dt)
    # vel = vel.at[2].add(tau / entity.inertia * dt)

    pos, vel, has_scored, hit_y_left, hit_y_right, wall_bounces = wall_collision(
        pos, vel, entity.radius, table
    )

    return jnp.concatenate([pos, vel]), has_scored, hit_y_left, hit_y_right, wall_bounces


@jax.jit
def physics_step(state, control, params: PhysicsParams, dt):
    """
    Performs one physics step for the entire 18D state and 6D control, including wall collisions and robot-puck collisions.

    Parameters:
    - state: (18,) array containing [r1_pos(3), r2_pos(3), puck_pos(3), r1_vel(3), r2_vel(3), puck_vel(3)]
    - control: (6,) array containing [r1_Fx, r1_Fy, r1_tau, r2_Fx, r2_Fy, r2_tau]
    - params: PhysicsParams dataclass containing table, robot, and puck parameters
    - dt: time step for integration

    Returns:
    - state_next: (18,) array of next state after applying control and physics
    - r1_has_collision: boolean indicating if Robot 1 collided with puck
    - r2_has_collision: boolean indicating if Robot 2 collided with puck
    - has_scored: 0 if no goal, 1 if scored on left goal, 2 if scored on right goal
    - hit_y: y-coordinate of wall hit (or -1 if no wall hit)
    - wall_bounces: (6,) array indicating which walls were bounced on (TopLeft, T, TR, BottomRight, B, BL)
    """

    r1_state = state[:6]  # Robot 1 (right)
    r2_state = state[6:12]  # Robot 2 (left)
    puck_state = state[12:18]


    u0_r1 = control[:3]
    u0_r2 = control[3:6]
    u0_r1 = jnp.clip(u0_r1, -params.robot.u_limits, params.robot.u_limits)
    u0_r2 = jnp.clip(u0_r2, -params.robot.u_limits, params.robot.u_limits)

    r1_state_next, _, _, _, _ = integrate_body(
        r1_state, u0_r1, params.robot, params.table, dt
    )

    r2_state_next, _, _, _, _ = integrate_body(
        r2_state, u0_r2, params.robot, params.table, dt
    )

    puck_state_next, has_scored, hit_y_left, hit_y_right, wall_bounces = integrate_body(
        puck_state,
        jnp.zeros(3),  # only robots are actuated
        params.puck,
        params.table,
        dt,
    )

    r1_pos = r1_state_next[:2]
    r1_vel = r1_state_next[3:5]

    r2_pos = r2_state_next[:2]
    r2_vel = r2_state_next[3:5]

    p_pos = puck_state_next[:2]
    p_vel = puck_state_next[3:5]

    # Robot1-puck collision
    r1_pos_new, r1_vel_new, p_pos_new, p_vel_new, r1_has_collision = (
        robot_puck_collision(r1_pos, r1_vel, p_pos, p_vel, params.robot, params.puck)
    )

    r1_state_next = r1_state_next.at[:2].set(r1_pos_new)
    r1_state_next = r1_state_next.at[3:5].set(r1_vel_new)

    # Robot2-puck collision, only one Robot can collide with puck per step
    r2_pos_new, r2_vel_new, p_pos_new, p_vel_new, r2_has_collision = (
        robot_puck_collision(
            r2_pos, r2_vel, p_pos_new, p_vel_new, params.robot, params.puck
        )
    )
    r2_state_next = r2_state_next.at[:2].set(r2_pos_new)
    r2_state_next = r2_state_next.at[3:5].set(r2_vel_new)
    puck_state_next = puck_state_next.at[:2].set(p_pos_new)
    puck_state_next = puck_state_next.at[3:5].set(p_vel_new)


    state_next = jnp.concatenate(
        [r1_state_next, r2_state_next, puck_state_next]
    )

    return (
        state_next,
        r1_has_collision,
        r2_has_collision,
        has_scored,
        hit_y_left,
        hit_y_right,
        wall_bounces,
    )
