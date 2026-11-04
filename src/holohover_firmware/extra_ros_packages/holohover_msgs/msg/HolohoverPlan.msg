# Plans of a planner (holohover_planner) for one hovercraft: one candidate per option it considers
# (shot_mpc: one per bounce sequence, always in the same order: direct, L, R, LR, RL for K = 2) and the
# one to follow. Published after every planning update, and with every candidate empty after a contact.

std_msgs/Header header                 # stamp: publication time; frame_id: table
string planner                         # e.g. shot_mpc
int32 chosen                           # index in candidates of the plan to follow; -1: none, the controller defends
string label                           # label of the chosen candidate, "" if none
HolohoverPlanCandidate[] candidates
