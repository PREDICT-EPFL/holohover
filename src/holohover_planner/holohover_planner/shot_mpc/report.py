"""Selector updates as plain data, and their solve-time summaries: python-sim's (timing_summary) and the
node's one line per shot (shot_summary)."""

import numpy as np


def update_json(log):
    """One selector update (ShotSelector.logs entry) as plain data."""
    return {"t": log.t, "outer": log.outer, "kind": log.kind, "parallel": log.parallel, "skip": log.skip,
            "chosen": log.chosen, "t_total": log.t_total, "t_wall": log.t_wall,
            "results": {k: {"valid": v[0], "cost": v[1], "tf": v[2], "t_wall": v[3], "iters": v[4], "cold": v[5],
                            "start": v[6]}
                        for k, v in log.results.items()}}


def timing_summary(updates):
    """Solve times of selector updates (update_json)."""
    solves = [r for u in updates for r in u["results"].values()]
    warm = np.array([r["t_wall"] for r in solves if not r["cold"]]) * 1e3
    cold = np.array([r["t_wall"] for r in solves if r["cold"]]) * 1e3
    outer = np.array([u["t_wall"] for u in updates if u["kind"] == "outer"]) * 1e3
    inner = np.array([u["t_wall"] for u in updates if u["kind"] != "outer"]) * 1e3  # inner and commit
    iters_warm = np.array([r["iters"] for r in solves if not r["cold"]])

    def row(name, a):
        if len(a) == 0:
            return f"  {name:<22}      -"
        return (f"  {name:<22} n={len(a):<5d} mean={a.mean():7.2f}  p95={np.percentile(a, 95):7.2f}  "
                f"max={a.max():7.2f} ms")

    lines = ["Solve times:", row("one B, warm start", warm), row("one B, cold (all seeds)", cold),
             row("outer update (all B)", outer), row("inner update (B*)", inner)]
    if len(iters_warm):
        lines.append(f"  warm Fatrop iterations: mean {iters_warm.mean():.1f}, max {iters_warm.max()}")
    return "\n".join(lines)


def ms_stats(name, seconds):
    """'name n=.. mean=.. max=..' of wall times in s, shown in ms; '' if there are none."""
    if len(seconds) == 0:
        return ""
    a = 1e3 * np.asarray(seconds)
    return f"{name} n={len(a)} mean={a.mean():.2f} max={a.max():.2f}"


def shot_summary(updates):
    """One line of the solve times of selector updates (update_json) [ms]: the updates (outer: all B,
    inner: B* or committed B*) and the solves of one B (warm or cold start, with the warm Fatrop iterations)."""
    solves = [r for u in updates for r in u["results"].values()]
    warm = [r for r in solves if not r["cold"]]
    parts = [ms_stats("outer", [u["t_wall"] for u in updates if u["kind"] == "outer"]),
             ms_stats("inner", [u["t_wall"] for u in updates if u["kind"] != "outer"]),
             ms_stats("warm", [r["t_wall"] for r in warm]),
             ms_stats("cold", [r["t_wall"] for r in solves if r["cold"]])]
    if warm:
        parts[2] += f" ({np.mean([r['iters'] for r in warm]):.1f} it)"
    return " | ".join(p for p in parts if p) + " [ms]"
