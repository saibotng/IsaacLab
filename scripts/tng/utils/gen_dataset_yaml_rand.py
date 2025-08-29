
"""
generate_cases_yaml_random.py

Generate an Isaac Sim case YAML with:
- Calibration episodes (same scheme as before):
  - 4 corners
  - + 4*g edges (g = --calibration-granularity), excluding corners
  - Each calibration episode pairs (x,y) with its opposite (-x,-y)
- PLUS a user-specified number of RANDOM episodes:
  - --num-episodes random episodes are added
  - For each random episode, object and target are sampled uniformly from the square
    [-dim, dim] x [-dim, dim], z=0.0
  - The Euclidean distance between object and target must be >= --threshold

Arguments:
  --dim <float>                         # Half-size of the square
  --calibration-granularity <int>       # g points PER EDGE (excluding corners)
  --num-episodes <int>                  # number of random episodes to add
  --threshold <float>                   # minimum allowed distance between object and target (meters)
  --seed <int>                          # (optional) RNG seed for reproducibility
  --out <path.yaml>                     # output YAML path

Notes:
- Total episodes = 4 + 4*g + num_episodes
- If sampling struggles to satisfy the threshold, the script will try many times and
  fail with a clear message suggesting to reduce --threshold or increase --dim.

Output YAML structure matches your example:
idle_case: {...}
test_cases:
  - id: case_001
    object: {pos: [x, y, 0.0], rpy: [0,0,0]}
    target: {pos: [x, y, 0.0], rpy: [0,0,0]}
    travel_height: 0.12
    prompt: "Pick up the blue cube and place it on the black platform"
"""

import argparse
import math
import random
from dataclasses import dataclass, asdict
from typing import List, Tuple
import yaml

PROMPT_TEXT = "Pick up the blue cube and place it on the black platform"
TRAVEL_HEIGHT = 0.12
RPY_ZERO = [0.0, 0.0, 0.0]
Z_CONST = 0.0


@dataclass
class Pose:
    pos: List[float]
    rpy: List[float]

@dataclass
class Case:
    id: str
    object: Pose
    target: Pose
    travel_height: float
    prompt: str


# ---------- Calibration helpers (same scheme as the first script) ----------

def corners(dim: float) -> List[Tuple[float, float]]:
    # Order chosen to match the earlier example
    return [
        ( dim,  dim),
        (-dim,  dim),
        ( dim, -dim),
        (-dim, -dim),
    ]

def opposite_point(x: float, y: float) -> Tuple[float, float]:
    return (-x, -y)

def edge_positions(dim: float, g: int) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    """
    Points along edges excluding corners.
    horizontal_edge_points: y=+dim and y=-dim (x varies)
    vertical_edge_points:   x=+dim and x=-dim (y varies)
    Evenly spaced at fractions k/(g+1), k=1..g.
    """
    if g <= 0:
        return [], []
    xs = [-dim + (2.0 * dim) * (k / (g + 1)) for k in range(1, g + 1)]
    ys = [-dim + (2.0 * dim) * (k / (g + 1)) for k in range(1, g + 1)]

    horizontal = [(x, +dim) for x in xs] + [(x, -dim) for x in xs]
    vertical   = [(+dim, y) for y in ys] + [(-dim, y) for y in ys]
    return horizontal, vertical

def make_pose(x: float, y: float) -> Pose:
    return Pose(pos=[float(x), float(y), Z_CONST], rpy=RPY_ZERO.copy())


# ---------- Random episode generation ----------

def sample_point(dim: float) -> Tuple[float, float]:
    """Uniform sample in the square [-dim, dim]^2."""
    return (random.uniform(-dim, dim), random.uniform(-dim, dim))

def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)

def generate_random_episodes(dim: float, n: int, threshold: float, max_trials_per_episode: int = 10000) -> List[Tuple[Tuple[float,float], Tuple[float,float]]]:
    """
    Rejection-sample n episodes with object and target in [-dim, dim]^2 and
    distance(object, target) >= threshold. Raises RuntimeError if it can't
    fulfill the request within the allowed trials.
    """
    episodes: List[Tuple[Tuple[float,float], Tuple[float,float]]] = []
    if threshold < 0:
        raise ValueError("--threshold must be non-negative")

    max_possible = 2 * dim * math.sqrt(2.0)  # diagonal across the square (corner-to-corner)
    if threshold > max_possible:
        raise ValueError(f"--threshold={threshold} exceeds the maximum possible distance in the square ({max_possible:.6f}). "
                         f"Increase --dim or reduce --threshold.")

    for _ in range(n):
        ok = False
        for _trial in range(max_trials_per_episode):
            o = sample_point(dim)
            t = sample_point(dim)
            if dist_xy(o, t) >= threshold:
                episodes.append((o, t))
                ok = True
                break
        if not ok:
            raise RuntimeError(
                "Failed to sample a valid episode under the threshold within the trial limit. "
                "Try reducing --threshold, increasing --dim, or decreasing --num-episodes."
            )
    return episodes


# ---------- Build YAML ----------

def build_cases(dim: float, calib_g: int, num_random: int, threshold: float, seed: int | None) -> List[Case]:
    if seed is not None:
        random.seed(seed)

    test_cases: List[Case] = []

    # 1) Calibration: corners
    for (ox, oy) in corners(dim):
        tx, ty = opposite_point(ox, oy)
        test_cases.append(Case(
            id="",
            object=make_pose(ox, oy),
            target=make_pose(tx, ty),
            travel_height=TRAVEL_HEIGHT,
            prompt=PROMPT_TEXT
        ))

    # 2) Calibration: edges
    horiz, vert = edge_positions(dim, calib_g)
    for (ox, oy) in horiz + vert:
        tx, ty = opposite_point(ox, oy)
        test_cases.append(Case(
            id="",
            object=make_pose(ox, oy),
            target=make_pose(tx, ty),
            travel_height=TRAVEL_HEIGHT,
            prompt=PROMPT_TEXT
        ))

    # 3) Random episodes with distance threshold
    random_eps = generate_random_episodes(dim, num_random, threshold)
    for (o, t) in random_eps:
        test_cases.append(Case(
            id="",
            object=make_pose(o[0], o[1]),
            target=make_pose(t[0], t[1]),
            travel_height=TRAVEL_HEIGHT,
            prompt=PROMPT_TEXT
        ))

    # Assign IDs with zero-padding
    total = len(test_cases)
    width = max(3, int(math.ceil(math.log10(max(1, total + 1)))))
    for idx, case in enumerate(test_cases, start=1):
        case.id = f"case_{idx:0{width}d}"

    return test_cases


def build_yaml(name: str, dim: float, calib_g: int, num_random: int, threshold: float, seed: int | None):
    doc = {
        "name": name,
        "idle_case": {
            "id": "idle_case",
            "object": {"pos": [0.1, -0.1, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "target": {"pos": [-0.1, 0.05, 0.0], "rpy": [0.0, 0.0, 0.0]},
            "travel_height": 0.12,
            "prompt": "Idle",
        },
        "test_cases": []
    }

    cases = build_cases(dim, calib_g, num_random, threshold, seed)
    for c in cases:
        doc["test_cases"].append({
            "id": c.id,
            "object": asdict(c.object),
            "target": asdict(c.target),
            "travel_height": c.travel_height,
            "prompt": c.prompt
        })
    return doc


def main():
    ap = argparse.ArgumentParser(description="Generate YAML with calibration + randomized episodes under a distance threshold.")
    ap.add_argument("--name", type=str, required=True, help="Name of the dataset.")
    ap.add_argument("--dim", type=float, required=True, help="Half-size of square (x,y in [-dim, dim]).")
    ap.add_argument("--calibration-granularity", type=int, default=2, help="g points per edge (excluding corners).")
    ap.add_argument("--num-random-episodes", type=int, required=True, help="Number of additional random episodes to generate.")
    ap.add_argument("--threshold", type=float, required=True, help="Minimum distance between object and target (meters).")
    ap.add_argument("--seed", type=int, default=None, help="RNG seed for reproducibility.")
    ap.add_argument("--out_dir", type=str, required=True, help="Output YAML filepath.")
    args = ap.parse_args()

    if args.num_random_episodes < 0:
        raise ValueError("--num-random-episodes must be >= 0")

    doc = build_yaml(args.name, args.dim, args.calibration_granularity, args.num_random_episodes, args.threshold, args.seed)

    with open(args.out_dir + args.name + ".yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)

    calib_total = 4 + 4 * max(0, args.calibration_granularity)
    total = calib_total + args.num_random_episodes
    print(f"Wrote {args.out_dir + args.name + '.yaml'}")
    print(f"Calibration episodes: {calib_total} (4 corners + {4*max(0,args.calibration_granularity)} edges)")
    print(f"Random episodes:      {args.num_random_episodes}  (threshold = {args.threshold})")
    print(f"TOTAL episodes:       {total}")


if __name__ == "__main__":
    main()
