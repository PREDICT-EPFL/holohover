import jax
import jax.numpy as jnp

from dynamics import physics_step


def rollout(x0, control_seq, physics, dt):
    """
    Rollout from a given starting position using a pre-defined set of inputs at refresh rate dt.
    Used to create Path Plan when DIAL has finished cooking up an input sequence
    """
    def step(x, u):
        x_next, has_collision, _, _, _ = physics_step(x, u, physics, dt)
        return x_next, (x_next, has_collision)
    _, (trajectory, collisions_detected) = jax.lax.scan(step, x0, control_seq)
    return (trajectory, collisions_detected)

def rollout_pd(x0, target_pos, H, physics, dt):
    """
    Rollout from a given starting position using a PD-controller tracking a fixed target state.
    Used to warm-start simulation (robot set to track future puck position)
    """
    def step(x, _):
        kp = 10
        kd = 2
        u = kp * (target_pos - x[:3]) + kd * (jnp.zeros(3) - x[3:6])
        # u = kp * (x[6:9] - x[:3]) + kd * (x[9:] - x[3:6])
        x_next, _, _, _, _ = physics_step(x, u, physics, dt)
        return x_next, (x, u)
    _, (trajectory, inputs) = jax.lax.scan(step, x0, length=H)
    return trajectory, inputs

def rollout_controller(x0, controller, physics, H_max, dt):
    """
    Rollout from a given initial state with a given controller (defined as u = f(state)).
    Simulate only until a goal is scored.
    
    Used in DIAL cost calulation to see if shot went in.
    """
    def step(carry, _):
        x, has_scored, hit_y, wall_bounce_sequence, bounce_idx = carry
        u0 = controller(x)

        def scored_branch(_): # do nothing
            return x, 0, has_scored, hit_y, jnp.zeros(6)

        def normal_branch(_):
            # Regular physics step
            x_next, _, has_scored_next, hit_y_next, wall_bounces_next = physics_step(x, u0, physics, dt)
            return x_next, 0, has_scored_next, hit_y_next, wall_bounces_next

        hit_wall = hit_y != -1.0 # magic number defined in dynamics
        trajectory, _, has_scored, hit_y, wall_bounces = jax.lax.cond(
            hit_wall, scored_branch, normal_branch, operand=None
            )
        
        # Extract wall indices from wall bounces
        wall_indices = jnp.where(wall_bounces > 0, size=2, fill_value=-1)[0]
        has_bounce = jnp.any(wall_bounces > 0)
        
        # Store
        wall_bounce_sequence = wall_bounce_sequence.at[bounce_idx].set(
            jnp.where(has_bounce, wall_indices[0], -1)
        )
        wall_bounce_sequence = wall_bounce_sequence.at[bounce_idx + 1].set(
            jnp.where(has_bounce & (wall_indices[1] != -1), wall_indices[1], -1)
        )
        bounce_idx = bounce_idx + jnp.where(has_bounce, 
                             jnp.sum(wall_bounces > 0), 
                             0)

        return (trajectory, has_scored, hit_y, wall_bounce_sequence, bounce_idx), x
    
    wall_bounce_sequence = jnp.full(10, -1, dtype=jnp.int32)  # Pre-allocate with -1 as padding, also very naughty hardcoding
    (_, has_scored, hit_y, wall_bounce_sequence, _), trajectory = jax.lax.scan(
        step, (x0, 0, -1.0, wall_bounce_sequence, 0), length=H_max
    )
    return trajectory, has_scored, hit_y, wall_bounce_sequence

def rollout_dial(x0, input_seq, tf, Q, R, physics, H, def_controller, H_goal_check, dt_sim):
    """
    Rollout a starting position with given inputs while computing the costs.
    Used inside the DIAL function to evaluate cost of all randomly generate trajectories.
    """
    dt = tf / H

    def step(carry, u):
        x, u_prev = carry
        x_next, has_puck_collision, _, _, _ = physics_step(x, u, physics, dt)
        r_pos = x_next[:2]
        r_vel = x_next[3:5]
        p_pos = x_next[6:8]
        p_vel = x_next[9:11]

        cost_du = 1e3*(u - u_prev) @ R @ (u - u_prev)
        # cost_du = 0

        cost_u = 1e2* u @ R @ u
        # cost_u = 0

        cost_dist = jnp.linalg.norm(r_pos - p_pos)**2
        # cost_dist = 0

        cost_side = jnp.where(r_pos[0]<=0.5*physics.table.width, 1e9, 0)
        # cost_side = 0

        cost = cost_side + cost_du + cost_dist + cost_u
        return (x_next, u), (x_next, cost, has_puck_collision)
    
    carry_init = (x0, jnp.zeros_like(input_seq[0]))
    (x_final, _), (trajectory, costs, puck_collisions) = jax.lax.scan(step, carry_init, input_seq)

    total_cost = jnp.sum(costs)

    total_cost += jnp.where(tf>0.15, 0, 1e10) # keep tf positive
    
    # Keep paths from becoming too short -> avoid jiggly behavior near puck
    enforce_last_step_collision = dt > dt_sim
    tf_cost = 1e3 * tf
    last_step_collision_cost = jnp.where(jnp.all(puck_collisions[:-1]==0) & (puck_collisions[-1]==1), 0, 1e10)
    any_step_collision_cost = jnp.where(jnp.any(puck_collisions==1), 0, 1e10)
    total_cost += jnp.where(enforce_last_step_collision,
                            tf_cost+last_step_collision_cost,
                            any_step_collision_cost)

    # Puck going to goal
    _, will_score, hit_y, wall_bounce_sequence = rollout_controller(x_final, def_controller, physics, H_goal_check, dt_sim)
    # wall_bounce_sequence = wall_bounce_sequence.astype(jnp.int32)

    # total_cost += jnp.where(will_score==1, 0, 1e10) # 0: no goal, 1: goal left, 2: goal right
    goal_y = 0.5*physics.table.height
    total_cost += 1e4 * jnp.linalg.norm(hit_y - goal_y)
    hit_wall = hit_y != -1.0
    total_cost += jnp.where(hit_wall, 0, 1e10)

    wall_bounce_desired = jnp.array([0, 1, 0, 0, 0, 0]).astype(jnp.int32)
    # total_cost += jnp.where((wall_bounce_counter[4]==1) & (jnp.sum(wall_bounce_counter)==1), 0, 1e10) # bounce off bottom wall only
    total_cost += jnp.where((wall_bounce_sequence[0]==4) & (wall_bounce_sequence[1]==-1), 0, 1e10)

    # Old alignment stuff:
    # r_pos_final = x_final[:2]
    # p_pos_final = x_final[6:8]
    # p_vel_final = x_final[9:11]
    # puck_to_goal = physics.table.left_goal - p_pos_final
    # goal_dir = puck_to_goal / (jnp.linalg.norm(puck_to_goal) + 1e-10)
    # goal_dir = puck_to_goal / (jnp.linalg.norm(puck_to_goal) + 1e-10)
    # vel_dir = p_vel_final / (jnp.linalg.norm(p_vel_final) + 1e-10)
    # alignment = jnp.dot(goal_dir, vel_dir)

    # total_cost += 1e7 * (1 - alignment) # go toward goal
    # total_cost += 1e5 * jnp.linalg.norm(r_pos_final - p_pos_final)**2


    return trajectory, total_cost, wall_bounce_sequence

