# One candidate plan of a HolohoverPlan: the motion of the hovercraft up to its impact with the puck.
# Empty (no states, cost NaN) when the planner has no valid plan for this option.

string label                           # e.g. the bounce sequence: direct, L, R, LR, RL
float64 cost                           # planner cost (shot_mpc: inner cost J*(B), the selection criterion); NaN if empty
builtin_interfaces/Time t0             # time of node 0
float64 tf                             # duration from t0 to the impact [s]; grid step tf / N

HolohoverState[] states                # N + 1 nodes on the grid, table frame; yaw = w_z = 0 (no heading)
float64[] acc_x                        # N zero-order-hold accelerations, table frame [m/s^2]
float64[] acc_y

geometry_msgs/Point impact_puck        # puck centre at the impact, table frame
geometry_msgs/Vector3 puck_velocity    # puck velocity just after the impact, table frame
