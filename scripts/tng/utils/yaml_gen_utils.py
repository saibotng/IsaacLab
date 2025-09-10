import math
from typing import List, Tuple, Optional
import numpy as np
import random
from dataclasses import dataclass, asdict
from colors import Color
from scipy.spatial.transform import Rotation as R
from enum import Enum

WEDGE_COUNT = 72     # angular wedges
LEN_BINS    = 60
MAX_DISTANCE_LIMIT_FACTOR = 1/math.sqrt(2)
FIXED_TARGET_XY = (0.0, 0.0)  # radians
PROMPT_TEMPLATE = "Pick up the {object_color} cube and place it on the {target_color} platform"
DEFAULT_TRAVEL_HEIGHT = 0.12
RPY_ZERO = [0.0, 0.0, 0.0]
MAX_GRIPPER_DISPLACEMENT = 0.038
NUM_ARM_JOINTS = 6

@dataclass
class Pose:
    pos: List[float]
    rpy: List[float]

class CameraPose(Enum):
    CAMERA_FRONT_POSE = Pose(pos=[1.5, 0.0, 0.8], rpy=[0.0, 55.0, 90.0])
    CAMERA_SIDE_POSE = Pose(pos=[0.5, 0.9, 0.5], rpy=[-60, 0.0, 180.0])
    CAMERA_WRIST_POSE = Pose(pos=[0.0, -0.11, 0.035], rpy=[151.0, 0.0, 0.0]) 


    @property
    def pose(self):
        return self.value

    @property
    def pretty(self) -> str:
        """Readable name, e.g. 'Deep Purple' for Color.DEEP_PURPLE."""
        return self.name.replace("_", " ").lower()
    
    @classmethod
    def get_camera_pose_from_string(cls, name: str) -> Pose:
        try:
            _ = cls[name]
        except KeyError:
            raise ValueError(f"Unknown camera pose name: {name}. Valid names are: {[pose.name for pose in cls]}")
        return cls[name].pose
    
    @classmethod
    def apply_camera_pose_randomization(cls, base_pose: Pose, position_jitter: float, rotation_jitter: float) -> Pose:
        """Apply random jitter to a given camera pose."""
        # Use global random state since seed is already set in calling scripts
        # Creating a new Random(seed) instance would produce identical results for each call

        # Apply position jitter
        jittered_pos = [
            base_pose.pos[0] + random.uniform(-position_jitter, position_jitter),
            base_pose.pos[1] + random.uniform(-position_jitter, position_jitter),
            base_pose.pos[2] + random.uniform(-position_jitter, position_jitter),
        ]

        # Apply rotation jitter
        r = R.from_euler('xyz', base_pose.rpy, degrees=True)
        jitter_rot = R.from_euler('xyz', [
            random.uniform(-rotation_jitter, rotation_jitter),
            random.uniform(-rotation_jitter, rotation_jitter),
            random.uniform(-rotation_jitter, rotation_jitter),
        ], degrees=True)
        new_r = (jitter_rot * r).as_euler('xyz', degrees=True)

        return Pose(pos=jittered_pos, rpy=new_r.tolist())




@dataclass
class Case:
    id: str
    object: Pose
    target: Pose
    object_rgb: Tuple[float, float, float] = Color.BLUE.rgb # Default blue
    target_rgb: Tuple[float, float, float] = Color.BLACK.rgb # Default black
    table_offset: Pose = Pose(pos=[0.0, 0.0, 0.0], rpy=[0.0, 0.0, 0.0])
    robot_joint_offsets: Tuple[float, float, float, float, float, float] = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    gripper_offset: Tuple[float] = (0.0,)
    prompt: str = PROMPT_TEMPLATE.format(object_color=Color.BLUE.pretty, target_color=Color.BLACK.pretty)
    camera_pose_main: Pose = CameraPose.CAMERA_FRONT_POSE.pose
    camera_pose_secondary: Pose = CameraPose.CAMERA_SIDE_POSE.pose
    camera_pose_wrist: Pose = CameraPose.CAMERA_WRIST_POSE.pose


@dataclass
class BenchCase(Case):
    pass    

@dataclass
class DatasetCase(Case):
    travel_height: float = DEFAULT_TRAVEL_HEIGHT

def sample_yaw_angles_for_poses(poses: List[Pose], yaw_range: float) -> List[Pose]:
    for p in poses:
        yaw = random.uniform(-yaw_range, yaw_range)
        p.rpy[2] = yaw
    return poses

def sample_table_offset(max_z_offset: float) -> Pose:
    """Sample a random table offset within the given max_offset."""
    tz = random.uniform(-max_z_offset, max_z_offset)
    return Pose(pos=[0.0, 0.0, tz], rpy=[0.0, 0.0, 0.0])

def sample_robot_starting_pose_offsets(joint_randomization_range: float) -> Tuple[List[float], List[float]]:
    joint_limits = [(-joint_randomization_range, joint_randomization_range)]*NUM_ARM_JOINTS
    joint_positions = [random.uniform(lim[0], lim[1]) for lim in joint_limits]
    gripper_position = [random.uniform(0.0, MAX_GRIPPER_DISPLACEMENT)]
    return joint_positions, gripper_position

def sample_point(dim: float) -> Tuple[float, float]:
    """Uniform sample in the square [-dim, dim]^2."""
    return (random.uniform(-dim, dim), random.uniform(-dim, dim))

def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    return math.hypot(dx, dy)


def gen_random_pairs(dim: float, n: int, fixed_target: bool, threshold: float, max_trials_per_episode: int = 10000) -> List[Tuple[Tuple[float,float], Tuple[float,float]]]:
    """
    Rejection-sample n episodes with object and target in [-dim, dim]^2 and
    distance(object, target) >= threshold. Raises RuntimeError if it can't
    fulfill the request within the allowed trials.
    """
    pairs: List[Tuple[Tuple[float,float], Tuple[float,float]]] = []
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
            t = sample_point(dim) if not fixed_target else FIXED_TARGET_XY
            if dist_xy(o, t) >= threshold:
                pairs.append((o, t))
                ok = True
                break
        if not ok:
            raise RuntimeError(
                "Failed to sample a valid episode under the threshold within the trial limit. "
                "Try reducing --threshold, increasing --dim, or decreasing --num-episodes."
            )
    return pairs

def get_corners_with_margin(dim: float, relative_margin: float) -> List[Tuple[float, float]]:
    # Order chosen to match your example:
    # (dim, dim), (-dim, dim), (dim, -dim), (-dim, -dim)
    return [
        ( dim * (1+relative_margin),  dim * (1+relative_margin)),
        (-dim * (1+relative_margin),  dim * (1+relative_margin)),
        ( dim * (1+relative_margin), -dim * (1+relative_margin)),
        (-dim * (1+relative_margin), -dim * (1+relative_margin)),
    ]

def grid_centers(dim: float, r: int) -> np.ndarray:
    """Centers of an r×r grid over [-dim, dim]^2, excluding the borders."""
    if r <= 0:
        return np.empty((0, 2), dtype=float)
    step = (2.0 * dim) / r
    coords = [-dim + (i + 0.5) * step for i in range(r)]
    centers = [(x, y) for y in coords for x in coords]  # row-major (y outer for visual grouping)
    return np.array(centers, dtype=float)

def grid_points(dim: float, r: int) -> np.ndarray:
    """Centers of an r×r grid over [-dim, dim]^2, excluding the borders."""
    if r <= 0:
        return np.empty((0, 2), dtype=float)
    step = (2.0 * dim) / (r-1)
    coords = [-dim + i * step for i in range(r)]
    grid_points = [(x, y) for y in coords for x in coords]  # row-major (y outer for visual grouping)
    return np.array(grid_points, dtype=float)

def build_pairs(candidates: np.ndarray, threshold: float) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:

    N = candidates.shape[0]
    if N == 0:
        return []

    # Quick upper bound on possible distance (box corners)
    minx, miny = candidates.min(axis=0)
    maxx, maxy = candidates.max(axis=0)
    dmax = math.hypot(maxx - minx, maxy - miny) * MAX_DISTANCE_LIMIT_FACTOR

    if threshold >= dmax - 1e-12:
        raise ValueError("threshold too large: no feasible edges exist")

    # Build candidate edges and bucket by (wedge, length_bin)
    buckets: dict[Tuple[int, int], List[Tuple[int, int]]] = {}
    for i in range(N):
        xi, yi = candidates[i]
        has_edge = False
        for j in range(N):
            if i == j:
                continue
            xj, yj = candidates[j]
            dx, dy = (xj - xi), (yj - yi)
            d = math.hypot(dx, dy)
            if d < threshold or d > dmax:
                continue
            # angle wedge in [0, WEDGE_COUNT-1]
            ang = math.atan2(dy, dx)
            w = int(((ang + math.pi) / (2.0 * math.pi)) * WEDGE_COUNT)
            if w >= WEDGE_COUNT:
                w = WEDGE_COUNT - 1
            # length bin in [0, LEN_BINS-1]
            t = (d - threshold) / max(1e-12, (dmax - threshold))
            lb = int(t * LEN_BINS)
            if lb >= LEN_BINS:
                lb = LEN_BINS - 1
            buckets.setdefault((w, lb), []).append((i, j))
            has_edge = True
        if not has_edge:
            # This source has no feasible targets at this threshold
            raise ValueError(f"Source {i} has no feasible targets; lower the threshold.")

    # Shuffle within buckets, then interleave buckets round-robin to reduce bias
    for key in buckets:
        random.shuffle(buckets[key])
    keys = sorted(buckets.keys())  # deterministic bucket order; contents are shuffled
    interleaved: List[Tuple[int, int]] = []
    remaining = sum(len(v) for v in buckets.values())
    idx = 0
    while remaining > 0:
        k = keys[idx]
        if buckets[k]:
            interleaved.append(buckets[k].pop())
            remaining -= 1
        idx = (idx + 1) % len(keys)

    # Build per-source adjacency list in the interleaved order (avoid duplicates)
    adj: List[List[int]] = [[] for _ in range(N)]
    for i, j in interleaved:
        if not adj[i] or adj[i][-1] != j:
            if j not in adj[i]:
                adj[i].append(j)

    # Greedy pass
    match_src = [-1] * N      # target index chosen for each source (or -1)
    match_tgt = [-1] * N      # source matched to each target (or -1)
    for i, j in interleaved:
        if match_src[i] == -1 and match_tgt[j] == -1:
            match_src[i] = j
            match_tgt[j] = i

    # Augmenting-path DFS to complete to a perfect matching (if feasible)
    def dfs(u: int, seen: List[bool]) -> bool:
        for v in adj[u]:
            if seen[v]:
                continue
            seen[v] = True
            if match_tgt[v] == -1 or dfs(match_tgt[v], seen):
                match_tgt[v] = u
                match_src[u] = v
                return True
        return False

    for u in range(N):
        if match_src[u] == -1:
            seen = [False] * N
            if not dfs(u, seen):
                # No augmenting path found => no perfect matching under this threshold
                raise ValueError(
                    "Could not complete a perfect assignment with the given threshold. "
                    "Try lowering the threshold slightly."
                )

    # Build (object, target) coordinate pairs
    pairs: List[Tuple[Tuple[float, float], Tuple[float, float]]] = []
    for i in range(N):
        j = match_src[i]
        ox, oy = candidates[i]
        tx, ty = candidates[j]
        pairs.append(((float(ox), float(oy)), (float(tx), float(ty))))
    return pairs

