"""Selector updates as plain data (saved in the episode log, read by viz/foxglove_viz.py) and the
solve-time summary printed after a run."""

import numpy as np


def update_json(log):
    """One selector update (ShotSelector.logs entry) as plain data."""
    return {"t": log.t, "outer": log.outer, "kind": log.kind, "parallel": log.parallel, "skip": log.skip,
            "chosen": log.chosen, "t_total": log.t_total, "t_wall": log.t_wall,
            "results": {k: {"valid": v[0], "cost": v[1], "score": v[2], "tf": v[3],
                            "t_wall": v[4], "iters": v[5], "cold": v[6], "start": v[7]}
                        for k, v in log.results.items()}}


def timing_summary(updates, tracker_times=()):
    """Solve times of selector updates (update_json) and tracker solves (s)."""
    solves = [r for u in updates for r in u["results"].values()]
    warm = np.array([r["t_wall"] for r in solves if not r["cold"]]) * 1e3
    cold = np.array([r["t_wall"] for r in solves if r["cold"]]) * 1e3
    outer = np.array([u["t_wall"] for u in updates if u["kind"] == "outer"]) * 1e3
    inner = np.array([u["t_wall"] for u in updates if u["kind"] != "outer"]) * 1e3  # inner and commit
    iters_warm = np.array([r["iters"] for r in solves if not r["cold"]])
    tracker = np.array(tracker_times) * 1e3

    def row(name, a):
        if len(a) == 0:
            return f"  {name:<22}      -"
        return (f"  {name:<22} n={len(a):<5d} mean={a.mean():7.2f}  p95={np.percentile(a, 95):7.2f}  "
                f"max={a.max():7.2f} ms")

    lines = ["Solve times:", row("one B, warm start", warm), row("one B, cold (all seeds)", cold),
             row("outer update (all B)", outer), row("inner update (B*)", inner),
             row("tracker (ref + QP)", tracker)]
    if len(iters_warm):
        lines.append(f"  warm IPOPT iterations: mean {iters_warm.mean():.1f}, max {iters_warm.max()}")
    return "\n".join(lines)
