"""
generate_cases_yaml.py

Generate an Isaac Sim case YAML with:
- 4 corner calibration episodes + edge calibration episodes controlled by --calibration-granularity
- Additional grid episodes controlled by --resolution
- Object/target constrained to a square: x,y in [-dim, dim], z=0.0
- Opposite-edge pairing for calibration (object corner/edge vs. opposite corner/edge as target)
- Grid episodes: one episode for every ordered pair of distinct grid-cell centers (N*(N-1))

Usage:
  python generate_cases_yaml.py --dim 0.2 --calibration-granularity 2 --resolution 3 --out cases.yaml

Notes:
- "calibration granularity" (g) means: g evenly spaced points per edge, excluding corners.
  For each such edge point we create ONE episode with the target at the *opposite* edge point (x,y)->(-x,-y).
  Total calibration episodes = 4 (corners) + 4*g (edges).
- Grid centers are the centers of an r×r grid (r = --resolution) over [-dim, dim]^2, excluding z (always 0).
  Grid episodes = (r*r) * ((r*r) - 1).
- The order of episodes is: corners → edges → grid. IDs are zero-padded based on total count.

The produced YAML matches the structure you showed:
idle_case: {...}
test_cases:
  - id: case_001
    object: {pos: [x, y, 0.0], rpy: [0.0, 0.0, 0.0]}
    target: {pos: [x, y, 0.0], rpy: [0.0, 0.0, 0.0]}
    travel_height: 0.12
    prompt: "Pick up the blue cube and place it on the black platform"
"""

import argparse
import math
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


def edge_positions(dim: float, g: int) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
    """
    Return two lists:
      horizontal_edge_points: points on y=+dim and y=-dim (excluding corners)
      vertical_edge_points:   points on x=+dim and x=-dim (excluding corners)
    Each list is ordered from negative to positive along the varying axis.
    """
    if g <= 0:
        return [], []

    # g evenly spaced points EXCLUDING corners: k=1..g at fraction k/(g+1)
    # For y=±dim edges, x varies; for x=±dim edges, y varies.
    xs = [-dim + (2.0 * dim) * (k / (g + 1)) for k in range(1, g + 1)]
    ys = [-dim + (2.0 * dim) * (k / (g + 1)) for k in range(1, g + 1)]

    horizontal = []  # (x,y) on top and bottom edges
    for x in xs:
        horizontal.append((x, +dim))  # top edge
    for x in xs:
        horizontal.append((x, -dim))  # bottom edge

    vertical = []  # (x,y) on right and left edges
    for y in ys:
        vertical.append((+dim, y))  # right edge
    for y in ys:
        vertical.append((-dim, y))  # left edge

    return horizontal, vertical


def opposite_point(x: float, y: float) -> Tuple[float, float]:
    return (-x, -y)


def corners(dim: float) -> List[Tuple[float, float]]:
    # Order chosen to match your example:
    # (dim, dim), (-dim, dim), (dim, -dim), (-dim, -dim)
    return [
        ( dim,  dim),
        (-dim,  dim),
        ( dim, -dim),
        (-dim, -dim),
    ]


def grid_centers(dim: float, r: int) -> List[Tuple[float, float]]:
    """Centers of an r×r grid over [-dim, dim]^2, excluding the borders."""
    if r <= 0:
        return []
    step = (2.0 * dim) / r
    coords = [-dim + (i + 0.5) * step for i in range(r)]
    centers = [(x, y) for y in coords for x in coords]  # row-major (y outer for visual grouping)
    return centers


def make_pose(x: float, y: float) -> Pose:
    return Pose(pos=[float(x), float(y), Z_CONST], rpy=RPY_ZERO.copy())


def build_cases(dim: float, calib_g: int, resolution: int) -> List[Case]:
    test_cases: List[Case] = []

    # 1) Calibration: corners
    for (ox, oy) in corners(dim):
        tx, ty = opposite_point(ox, oy)
        test_cases.append(Case(
            id="",  # fill later
            object=make_pose(ox, oy),
            target=make_pose(tx, ty),
            travel_height=TRAVEL_HEIGHT,
            prompt=PROMPT_TEXT
        ))

    # 2) Calibration: edges (g points per edge, excluding corners)
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

    # 3) Grid episodes (ordered pairs of distinct centers)
    centers = grid_centers(dim, resolution)
    n = len(centers)
    for i in range(n):
        for j in range(n):
            if i == j:
                continue  # must be distinct centers
            (ox, oy) = centers[i]
            (tx, ty) = centers[j]
            test_cases.append(Case(
                id="",
                object=make_pose(ox, oy),
                target=make_pose(tx, ty),
                travel_height=TRAVEL_HEIGHT,
                prompt=PROMPT_TEXT
            ))

    # Fill IDs with zero padding
    total = len(test_cases)
    width = max(3, int(math.ceil(math.log10(max(1, total + 1)))))
    for idx, case in enumerate(test_cases, start=1):
        case.id = f"case_{idx:0{width}d}"

    return test_cases


def build_yaml(name: str, dim: float, calib_g: int, resolution: int):
    # Idle case kept (you can adjust the coordinates here if you prefer)
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

    cases = build_cases(dim, calib_g, resolution)
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
    ap = argparse.ArgumentParser(description="Generate Isaac Sim YAML of pick-and-place episodes.")
    ap.add_argument("--name", type=str, required=True, help="Name of the dataset.")
    ap.add_argument("--dim", type=float, required=True, help="Half-size of square region (x,y in [-dim, dim]).")
    ap.add_argument("--calibration-granularity", type=int, default=2, help="g points per edge (excluding corners).")
    ap.add_argument("--resolution", type=int, default=0, help="Grid resolution r (r×r cells). 0 disables grid episodes.")
    ap.add_argument("--out_dir", type=str, required=True, help="Output YAML filepath.")
    args = ap.parse_args()

    doc = build_yaml(args.name, args.dim, args.calibration_granularity, args.resolution)
    with open(args.out_dir + args.name + ".yaml", "w", encoding="utf-8") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)

    # A quick summary on stdout
    total = len(doc["test_cases"])
    g = args.calibration_granularity
    r = args.resolution
    calib_total = 4 + (4 * max(0, g))
    grid_total = (r * r) * (r * r - 1) if r > 0 else 0
    print(f"Wrote {args.out_dir + args.name + '.yaml'}")
    print(f"Calibration episodes: {calib_total} (4 corners + {4*max(0,g)} edges)")
    print(f"Grid episodes:        {grid_total}")
    print(f"TOTAL episodes:       {total}")


if __name__ == "__main__":
    main()