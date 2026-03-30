import json
import pickle
from pathlib import Path

from simulation import run_sim
from animation import TrajecoryAnimation


def load_config(path="config/config0.json"):
    with open(path, "r") as f:
        return json.load(f)


def simulate():
    config = load_config()
    run_sim(config)


def get_latest_pkl():
    p = Path("simulation_data")
    pkl_files = list(p.glob("*.pkl"))

    if not pkl_files:
        raise FileNotFoundError(f"No .pkl files found")

    latest_file = max(pkl_files, key=lambda f: f.stat().st_mtime)
    return latest_file


def animate(path=None, export=None, mode="standard"):
    if path is None:
        path = get_latest_pkl()

    with open(path, "rb") as f:
        print(f"Animating from {path}")
        pkl_content = pickle.load(f)

    animation = TrajecoryAnimation(*pkl_content)

    if not export:
        animation.play(mode=mode)
    else:
        animation.export_animation(filename=export)


if __name__ == "__main__":
    simulate()
    # animate()
    animate(mode="dial")
    # animate(export="apple_apple_25Hz_example")
    # animate("simulation_data/20260322_172120.pkl")
