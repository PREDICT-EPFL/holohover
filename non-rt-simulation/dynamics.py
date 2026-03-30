import jax
import jax.numpy as jnp
from flax import struct


# Parameters
@struct.dataclass
class Table:
    width: float
    height: float
    goal_width: float
    friction: float
    defensive_position_rel: list

    @property
    def defensive_position(self):
        def_pos = jnp.array([self.width * self.defensive_position_rel[0], # x
                             self.height * self.defensive_position_rel[1], # y
                             0]) # phi
        return def_pos
    
    @property
    def left_goal(self):
        return jnp.array([0, 0.5*self.height])
    
    @property
    def right_goal(self):
        return jnp.array([self.width, 0.5*self.height])


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
        M = jnp.array([1/self.mass, 1/self.mass, 1/self.inertia])
        return jnp.block([[jnp.zeros((3, 3))], [jnp.diag(M)]])


@struct.dataclass
class PhysicsParams:
    table: Table
    robot: Entity
    puck: Entity


def wall_collision(pos, vel, radius, table: Table):
    x, y = pos[0], pos[1]
    vx, vy = vel[0], vel[1]

    # detect goal
    goal_y_min = 0.5*table.height - 0.5*table.goal_width
    goal_y_max = 0.5*table.height + 0.5*table.goal_width
    in_goal_y = (y >= goal_y_min) & (y <= goal_y_max)
    scored_left = (x - radius < 0.0) & in_goal_y
    scored_right = (x + radius > table.width) & in_goal_y

    # 0: hasnt scored, 1: scored on left goal, 2: scored on right goal
    has_scored = jnp.where(scored_left, 1, jnp.where(scored_right, 2, 0))
    hit_y = jnp.where((x-radius<0.0), y, -1.0)

    # detect wall bounces:
    wall_bounces = jnp.zeros(6) # [TopLeft, T, TR, BottomRight, B, BL]
    z6 = jnp.zeros(6)

    cond_top = y >= goal_y_max
    cond_bottom = y <= goal_y_min

    # X lower
    cond = x - radius < 0.0
    x = jnp.where(cond, 2*(radius) - x, x)
    vx = jnp.where(cond, -vx, vx)
    wall_bounces += jnp.where(cond & cond_top, jnp.array([1, 0,0,0,0,0]), z6)
    wall_bounces += jnp.where(cond & cond_bottom, jnp.array([0,0,0,0,0, 1]), z6)

    # X upper
    cond = x + radius > table.width
    x = jnp.where(cond, 2*(table.width - radius) - x, x)
    vx = jnp.where(cond, -vx, vx)
    wall_bounces += jnp.where(cond & cond_top, jnp.array([0,0, 1, 0,0,0]), z6)
    wall_bounces += jnp.where(cond & cond_bottom, jnp.array([0,0,0, 1, 0,0]), z6)

    # Y lower
    cond = y - radius < 0.0
    y = jnp.where(cond, 2*(radius) - y, y)
    vy = jnp.where(cond, -vy, vy)
    wall_bounces += jnp.where(cond, jnp.array([0,0,0,0, 1, 0]), z6)

    # Y upper
    cond = y + radius > table.height
    y = jnp.where(cond, 2*(table.height - radius) - y, y)
    vy = jnp.where(cond, -vy, vy)
    wall_bounces += jnp.where(cond, jnp.array([0, 1, 0,0,0,0]), z6)


    pos = jnp.array([x, y, pos[2]])
    vel = jnp.array([vx, vy, vel[2]])

    return pos, vel, has_scored, hit_y, wall_bounces


def robot_puck_collision(r_pos, r_vel,
                         p_pos, p_vel,
                         r_params, p_params):

    delta = r_pos - p_pos
    dist = jnp.linalg.norm(delta)
    min_dist = r_params.radius + p_params.radius

    def collide(_):

        m1, m2 = r_params.mass, p_params.mass

        v1_new = r_vel - (2*m2/(m1+m2)) * (
            jnp.dot(r_vel-p_vel, delta) / (dist**2)
        ) * delta

        v2_new = p_vel - (2*m1/(m1+m2)) * (
            jnp.dot(p_vel-r_vel, -delta) / (dist**2)
        ) * (-delta)

        n = delta / dist
        overlap = min_dist - dist
        correction = 0.5 * overlap * n

        r_pos_new = r_pos + correction
        p_pos_new = p_pos - correction

        return r_pos_new, v1_new, p_pos_new, v2_new, True

    def no_collision(_):
        return r_pos, r_vel, p_pos, p_vel, False

    return jax.lax.cond(
        (dist < min_dist) & (dist > 1e-6),
        collide,
        no_collision,
        operand=None
    )


def integrate_body(state, control, entity, table, dt):

    pos = state[:3]
    vel = state[3:]

    control = jnp.clip(control, -entity.u_limits, entity.u_limits)

    pos = pos + vel * dt
    vel = vel * (1 - table.friction * dt)

    Fx, Fy, tau = control
    vel = vel.at[0].add(Fx / entity.mass * dt)
    vel = vel.at[1].add(Fy / entity.mass * dt)
    # vel = vel.at[2].add(tau / entity.inertia * dt)

    pos, vel, has_scored, hit_y, wall_bounces = wall_collision(pos, vel, entity.radius, table)

    return jnp.concatenate([pos, vel]), has_scored, hit_y, wall_bounces


@jax.jit
def physics_step(state, control, params: PhysicsParams, dt):

    robot_state = state[:6]
    puck_state  = state[6:]

    robot_state_next, _, _, _ = integrate_body(
        robot_state, control,
        params.robot, params.table, dt
    )

    puck_state_next, has_scored, hit_y, wall_bounces = integrate_body(
        puck_state, jnp.zeros(3), # only robot is actuated
        params.puck, params.table, dt
    )

    r_pos = robot_state_next[:2]
    r_vel = robot_state_next[3:5]

    p_pos = puck_state_next[:2]
    p_vel = puck_state_next[3:5]

    r_pos_new, r_vel_new, p_pos_new, p_vel_new, has_collision = robot_puck_collision(
        r_pos, r_vel,
        p_pos, p_vel,
        params.robot,
        params.puck
    )

    robot_state_next = robot_state_next.at[:2].set(r_pos_new)
    robot_state_next = robot_state_next.at[3:5].set(r_vel_new)

    puck_state_next = puck_state_next.at[:2].set(p_pos_new)
    puck_state_next = puck_state_next.at[3:5].set(p_vel_new)
    return jnp.concatenate([robot_state_next, puck_state_next]), has_collision, has_scored, hit_y, wall_bounces


