"""
Dataset Card Generation Tool

This script generates a standardized dataset card visualization from robotics 
benchmark YAML files. It creates a 3x3 grid layout with the following structure:

Row 1: Object positions | Target positions | Object→Target connections
Row 2: X/Y distributions | Travel distances | Travel directions
Row 3: Joint/Gripper offsets | (Reserved) | (Reserved)

The tool automatically infers coordinate ranges and generates a single combined
visualization suitable for dataset documentation.

Usage:
    python generate_dataset_card.py --in benchmark.yaml [options]
    
Options:
    --show         Display plot interactively in addition to saving
    --bins N       Number of histogram bins (default: 30)
    --angle-bins N Number of direction histogram bins (default: 36)
    --pad RATIO    Padding for coordinate bounds (default: 0.05)
"""

import argparse
import os
import math
import sys
import yaml
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

# Import Color class and CameraPose for camera base positions
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)
from colors import Color
from yaml_gen_utils import CameraPose

# Set global font sizes
plt.rcParams.update({
    'font.size': 13,           # Base font size
    'axes.titlesize': 15,      # Subplot titles
    'axes.labelsize': 13,      # Axis labels
    'xtick.labelsize': 12,     # X-axis tick labels
    'ytick.labelsize': 12,     # Y-axis tick labels
    'legend.fontsize': 12,     # Legend
})


def infer_dim(points, pad=0.05):
    """Infer square bounds from points and add a padding margin."""
    if not points:
        return (-1, 1)  # fallback
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    m = max(max(abs(min(xs)), abs(max(xs))), max(abs(min(ys)), abs(max(ys))))
    d = m * (1.0 + pad)
    return (-d, d)


def load_positions(yaml_path):
    """Load object and target positions and additional data from YAML file."""
    with open(yaml_path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    cases = doc.get("test_cases", [])
    metadata = doc.get("metadata", {})

    obj_pts = []
    tgt_pts = []
    lines = []  # ( (ox,oy), (tx,ty) )
    obj_yaws = []  # yaw angles in degrees
    tgt_yaws = []  # yaw angles in degrees
    table_heights = []  # table height offsets (z-coordinate)
    joint_offsets = []  # robot joint offsets (6 values per case)
    gripper_offsets = []  # gripper offsets
    obj_colors = []  # object RGB colors
    tgt_colors = []  # target RGB colors
    camera_poses_main = []  # main camera poses
    camera_poses_secondary = []  # secondary camera poses
    camera_poses_wrist = []  # wrist camera poses

    for c in cases:
        ox, oy, _ = c["object"]["pos"]
        tx, ty, _ = c["target"]["pos"]
        
        # Extract yaw angles (convert from degrees to radians for math operations)
        obj_yaw = c["object"]["rpy"][2] if "rpy" in c["object"] else 0.0
        tgt_yaw = c["target"]["rpy"][2] if "rpy" in c["target"] else 0.0
        
        # Extract table height offset
        table_height = c["table_offset"]["pos"][2] if "table_offset" in c and "pos" in c["table_offset"] else 0.0
        
        # Extract joint offsets
        robot_joints = c.get("robot_joint_offsets", [0.0] * 6)
        gripper_offset = c.get("gripper_offset", [0.0])[0] if c.get("gripper_offset") else 0.0
        
        # Extract colors
        obj_rgb = tuple(c.get("object_rgb", [0.0, 0.0, 0.0]))
        tgt_rgb = tuple(c.get("target_rgb", [0.0, 0.0, 0.0]))
        
        # Extract camera poses
        cam_main = c.get("camera_pose_main", {})
        cam_secondary = c.get("camera_pose_secondary", {})
        cam_wrist = c.get("camera_pose_wrist", {})
        
        camera_poses_main.append({
            'pos': cam_main.get('pos', [0, 0, 0]),
            'rpy': cam_main.get('rpy', [0, 0, 0])
        })
        camera_poses_secondary.append({
            'pos': cam_secondary.get('pos', [0, 0, 0]),
            'rpy': cam_secondary.get('rpy', [0, 0, 0])
        })
        camera_poses_wrist.append({
            'pos': cam_wrist.get('pos', [0, 0, 0]),
            'rpy': cam_wrist.get('rpy', [0, 0, 0])
        })
        
        obj_pts.append((ox, oy))
        tgt_pts.append((tx, ty))
        lines.append(((ox, oy), (tx, ty)))
        obj_yaws.append(obj_yaw)
        tgt_yaws.append(tgt_yaw)
        table_heights.append(table_height)
        joint_offsets.append(robot_joints)
        gripper_offsets.append(gripper_offset)
        obj_colors.append(obj_rgb)
        tgt_colors.append(tgt_rgb)

    return (obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, 
            gripper_offsets, obj_colors, tgt_colors, camera_poses_main, camera_poses_secondary, 
            camera_poses_wrist, metadata)


def extract_coordinates(obj_pts, tgt_pts):
    """Extract X and Y coordinates from object and target points."""
    obj_xs = [p[0] for p in obj_pts] if obj_pts else []
    obj_ys = [p[1] for p in obj_pts] if obj_pts else []
    tgt_xs = [p[0] for p in tgt_pts] if tgt_pts else []
    tgt_ys = [p[1] for p in tgt_pts] if tgt_pts else []
    return obj_xs, obj_ys, tgt_xs, tgt_ys


def calculate_travel_metrics(obj_pts, tgt_pts):
    """Calculate travel distances and direction angles."""
    distances = []
    angles_rad = []
    
    for (ox, oy), (tx, ty) in zip(obj_pts, tgt_pts):
        dx = tx - ox
        dy = ty - oy
        dist = math.hypot(dx, dy)
        distances.append(dist)
        
        if dist > 0.0:  # Only include non-zero distances for angles
            ang = math.atan2(dy, dx)  # radians in [-pi, pi]
            angles_rad.append(ang)
    
    return distances, angles_rad


def setup_coordinate_histogram_data(obj_pts, tgt_pts):
    """Prepare data for combined X/Y coordinate histogram."""
    obj_xs, obj_ys, tgt_xs, tgt_ys = extract_coordinates(obj_pts, tgt_pts)
    
    # Calculate symmetric range for all coordinates
    all_coords = obj_xs + obj_ys + tgt_xs + tgt_ys
    if not all_coords:
        return [], [], [], (-1, 1)
    
    max_abs = max(abs(coord) for coord in all_coords)
    coord_range = (-max_abs, max_abs)
    
    # Prepare data and labels for side-by-side bars
    data_sets = []
    labels = []
    colors = []
    
    if obj_xs:
        data_sets.append(obj_xs)
        labels.append('Object X')
        colors.append('blue')
    if obj_ys:
        data_sets.append(obj_ys)
        labels.append('Object Y')
        colors.append('lightblue')
    if tgt_xs:
        data_sets.append(tgt_xs)
        labels.append('Target X')
        colors.append('red')
    if tgt_ys:
        data_sets.append(tgt_ys)
        labels.append('Target Y')
        colors.append('lightcoral')
    
    return data_sets, labels, colors, coord_range


def add_square_boundary(ax, xlim, ylim):
    """Add square boundary to plot if limits are equal and symmetric."""
    if xlim and ylim and abs(xlim[0]) == abs(xlim[1]) and abs(ylim[0]) == abs(ylim[1]):
        d = xlim[1]
        ax.plot([-d, d, d, -d, -d], [d, d, -d, -d, d], linewidth=1)


def setup_polar_plot(ax):
    """Configure polar plot settings."""
    ax.set_theta_zero_location("E")  # 0° at +x (east)
    ax.set_theta_direction(1)        # CCW positive
    ax.set_xticks([0, math.pi/2, math.pi, 3*math.pi/2])
    ax.set_xticklabels(["0°", "90°", "180°", "270°"])
    ax.grid(True, linewidth=0.3, alpha=0.4)


def create_scatter_plot_enhanced(points, yaws, table_heights, title, ax, xlim=None, ylim=None):
    """Create an enhanced scatter plot with yaw angles and table height color grading."""
    if points and yaws and table_heights:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        
        # Create color mapping based on table heights
        heights = np.array(table_heights)
        
        # Create scatter plot with color grading
        scatter = ax.scatter(xs, ys, c=heights, s=40, cmap='viridis', alpha=0.7)
        
        # Add colorbar for table heights
        plt.colorbar(scatter, ax=ax, label='Table height offset [m]')
        
        # Add yaw angle indicators as small arrows
        arrow_length = (xlim[1] - xlim[0]) * 0.02 if xlim else 0.02  # 2% of plot range
        for i, (x, y, yaw) in enumerate(zip(xs, ys, yaws)):
            # Convert yaw from degrees to radians
            # Add 90 degrees because yaw=0 means facing up (positive y), not right (positive x)
            yaw_rad = math.radians(yaw + 90.0)
            dx = arrow_length * math.cos(yaw_rad)
            dy = arrow_length * math.sin(yaw_rad)
            
            ax.arrow(x, y, dx, dy, head_width=arrow_length*0.3, head_length=arrow_length*0.2, 
                    fc='black', ec='black', alpha=0.8, linewidth=1)
    
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    
    if xlim and ylim:
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        add_square_boundary(ax, xlim, ylim)
    
    ax.grid(True, linewidth=0.3, alpha=0.4)
    return ax


def create_line_plot(lines, title, ax, xlim=None, ylim=None):
    """Create a line plot showing connections."""
    for (o, t) in lines:
        xs = [o[0], t[0]]
        ys = [o[1], t[1]]
        ax.plot(xs, ys, linewidth=0.7)
    
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    
    if xlim and ylim:
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        add_square_boundary(ax, xlim, ylim)
    
    ax.grid(True, linewidth=0.3, alpha=0.4)
    return ax


def create_coordinate_histogram(obj_pts, tgt_pts, args, ax):
    """Create combined X/Y coordinate distribution histogram."""
    data_sets, labels, colors, coord_range = setup_coordinate_histogram_data(obj_pts, tgt_pts)
    
    if data_sets:
        ax.hist(data_sets, bins=args.bins, range=coord_range, 
                label=labels, color=colors, alpha=0.7, 
                histtype='bar', rwidth=1.0)
        ax.legend()
    
    ax.set_title("(d) Combined X/Y coordinate distributions")
    ax.set_xlabel("Position [m]")
    ax.set_ylabel("Count")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    return ax


def create_distance_histogram(obj_pts, tgt_pts, args, ax):
    """Create travel distance distribution histogram."""
    distances, _ = calculate_travel_metrics(obj_pts, tgt_pts)
    
    if distances:
        ax.hist(distances, bins=args.bins, alpha=0.7, color='green')
    
    ax.set_title("(e) Travel distance distribution")
    ax.set_xlabel("Distance [m]")
    ax.set_ylabel("Count")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    return ax


def create_direction_plot(obj_pts, tgt_pts, args, ax):
    """Create unweighted direction distribution polar plot."""
    _, angles_rad = calculate_travel_metrics(obj_pts, tgt_pts)
    
    if angles_rad:
        ax.hist(angles_rad, bins=args.angle_bins, range=(-math.pi, math.pi))
    
    ax.set_title("(f) Travel direction distribution", va="bottom")
    setup_polar_plot(ax)
    return ax


def create_joint_offset_plot(joint_offsets, gripper_offsets, ax):
    """Create a bar plot showing mean and standard deviation of joint and gripper offsets."""
    if joint_offsets and gripper_offsets:
        # Convert to numpy arrays for easier computation
        joint_data = np.array(joint_offsets)  # Shape: (n_cases, 6)
        gripper_data = np.array(gripper_offsets)  # Shape: (n_cases,)
        
        # Calculate means and standard deviations
        joint_means = np.mean(joint_data, axis=0)
        joint_stds = np.std(joint_data, axis=0)
        gripper_mean = np.mean(gripper_data)
        gripper_std = np.std(gripper_data)
        
        # Create labels with proper joint names
        joint_names = [
            "base joint",
            "shoulde joint", 
            "elbow joint",
            "wrist 1 joint",
            "wrist 2 joint",
            "wrist 3 joint",
        ]
        
        # Create bar plot for joints only on primary axis
        joint_x_pos = np.arange(len(joint_names))
        joint_bars = ax.bar(joint_x_pos, joint_means, yerr=joint_stds, capsize=5, alpha=0.7, color='darkblue')
        
        ax.set_ylabel('Joint offset [°]', color='darkblue')
        ax.set_title("(g) Initial actuator offsets (mean ± std)")
        ax.set_xticks(np.arange(len(joint_names) + 1))  # Add space for gripper
        ax.set_xticklabels(joint_names + ['gripper'], rotation=45, ha='right')
        ax.grid(True, linewidth=0.3, alpha=0.4)
        ax.tick_params(axis='y', labelcolor='darkblue')
        
        # Add zero line for joints
        ax.axhline(y=0, color='darkblue', linestyle='-', linewidth=0.5, alpha=0.7)
        
        # Create secondary y-axis for gripper
        ax2 = ax.twinx()
        gripper_x_pos = len(joint_names)  # Position at the end
        gripper_bar = ax2.bar(gripper_x_pos, gripper_mean, yerr=gripper_std, capsize=5, alpha=0.7, color='darkorange')
        
        ax2.set_ylabel('Gripper offset [m]', color='darkorange')
        ax2.tick_params(axis='y', labelcolor='darkorange')
        
        # Add zero line for gripper
        ax2.axhline(y=0, color='darkorange', linestyle='-', linewidth=0.5, alpha=0.7)
        
        # Align the zero lines of both axes if possible
        # Get the y-limits and try to make them symmetric around zero
        joint_ylim = ax.get_ylim()
        gripper_ylim = ax2.get_ylim()
        
        joint_max = max(abs(joint_ylim[0]), abs(joint_ylim[1]))
        gripper_max = max(abs(gripper_ylim[0]), abs(gripper_ylim[1]))
        
        ax.set_ylim(-joint_max * 1.1, joint_max * 1.1)
        ax2.set_ylim(-gripper_max * 1.1, gripper_max * 1.1)
    
    return ax


def create_color_distribution_plot(obj_colors, tgt_colors, ax):
    """Create a bar chart showing the distribution of object and target colors."""
    from collections import Counter
    
    if not obj_colors and not tgt_colors:
        ax.text(0.5, 0.5, 'No color data', ha='center', va='center', fontsize=16, color='gray')
        ax.axis('off')
        return ax
    
    # Convert RGB tuples to color names
    obj_color_names = []
    tgt_color_names = []
    
    for rgb in obj_colors:
        try:
            color = Color.from_rgb(rgb)
            obj_color_names.append(color.pretty)
        except ValueError:
            # If RGB doesn't match any known color, use a generic label
            obj_color_names.append(f"RGB{rgb}")
    
    for rgb in tgt_colors:
        try:
            color = Color.from_rgb(rgb)
            tgt_color_names.append(color.pretty)
        except ValueError:
            tgt_color_names.append(f"RGB{rgb}")
    
    # Count occurrences
    obj_counts = Counter(obj_color_names)
    tgt_counts = Counter(tgt_color_names)
    
    # Get all unique colors across both distributions
    all_colors = sorted(set(obj_color_names + tgt_color_names))
    
    # Prepare data for grouped bar chart
    obj_values = [obj_counts.get(color, 0) for color in all_colors]
    tgt_values = [tgt_counts.get(color, 0) for color in all_colors]
    
    # Create bar chart
    x = np.arange(len(all_colors))
    width = 0.35
    
    bars1 = ax.bar(x - width/2, obj_values, width, label='Object', alpha=0.7, color='blue')
    bars2 = ax.bar(x + width/2, tgt_values, width, label='Target', alpha=0.7, color='red')
    
    ax.set_ylabel('Count')
    ax.set_title('(h) Object and target color distribution')
    ax.set_xticks(x)
    ax.set_xticklabels(all_colors, rotation=45, ha='right')
    ax.legend()
    ax.grid(True, linewidth=0.3, alpha=0.4, axis='y')
    
    return ax


def create_camera_pose_variation_plot(camera_poses_main, camera_poses_secondary, 
                                      camera_poses_wrist, metadata, ax, include_secondary=True):
    """Create a bar chart showing camera pose variations from base positions."""
    
    # Get base camera poses from metadata
    camera_main_name = metadata.get('camera_main', 'CAMERA_FRONT_POSE')
    camera_secondary_name = metadata.get('camera_secondary', 'CAMERA_SIDE_POSE')
    camera_wrist_name = metadata.get('camera_wrist', 'CAMERA_WRIST_POSE')
    
    # Get base poses
    try:
        base_main = CameraPose.get_camera_pose_from_string(camera_main_name)
        base_secondary = CameraPose.get_camera_pose_from_string(camera_secondary_name)
        base_wrist = CameraPose.get_camera_pose_from_string(camera_wrist_name)
    except ValueError as e:
        ax.text(0.5, 0.5, f'Error: {str(e)}', ha='center', va='center', fontsize=12, color='red')
        ax.axis('off')
        return ax
    
    # Calculate variations for each camera
    def calculate_variation(poses, base_pose):
        """Calculate translation distance and rotation angle from base pose."""
        translations = []
        rotations = []
        
        for pose in poses:
            # Translation: Euclidean distance
            pos_diff = np.array(pose['pos']) - np.array(base_pose.pos)
            translation = np.linalg.norm(pos_diff)
            translations.append(translation)
            
            # Rotation: angle difference using scipy Rotation
            r_current = R.from_euler('xyz', pose['rpy'], degrees=True)
            r_base = R.from_euler('xyz', base_pose.rpy, degrees=True)
            r_diff = r_current * r_base.inv()
            
            # Get rotation angle (magnitude of rotation)
            rotation_vec = r_diff.as_rotvec()
            rotation_angle = np.linalg.norm(rotation_vec) * 180.0 / np.pi  # Convert to degrees
            rotations.append(rotation_angle)
        
        return translations, rotations
    
    trans_main, rot_main = calculate_variation(camera_poses_main, base_main)
    trans_secondary, rot_secondary = calculate_variation(camera_poses_secondary, base_secondary)
    trans_wrist, rot_wrist = calculate_variation(camera_poses_wrist, base_wrist)
    
    # Calculate statistics - filter based on include_secondary
    if include_secondary:
        trans_means = [np.mean(trans_main), np.mean(trans_secondary), np.mean(trans_wrist)]
        trans_stds = [np.std(trans_main), np.std(trans_secondary), np.std(trans_wrist)]
        rot_means = [np.mean(rot_main), np.mean(rot_secondary), np.mean(rot_wrist)]
        rot_stds = [np.std(rot_main), np.std(rot_secondary), np.std(rot_wrist)]
        colors = ['steelblue', 'coral', 'mediumseagreen']
        camera_type_names = [
            camera_main_name.replace('CAMERA_', '').replace('_POSE', '').title(),
            camera_secondary_name.replace('CAMERA_', '').replace('_POSE', '').title(),
            camera_wrist_name.replace('CAMERA_', '').replace('_POSE', '').title()
        ]
        camera_labels = [
            f'Main ({camera_type_names[0]})',
            f'Secondary ({camera_type_names[1]})',
            f'Wrist ({camera_type_names[2]})'
        ]
    else:
        trans_means = [np.mean(trans_main), np.mean(trans_wrist)]
        trans_stds = [np.std(trans_main), np.std(trans_wrist)]
        rot_means = [np.mean(rot_main), np.mean(rot_wrist)]
        rot_stds = [np.std(rot_main), np.std(rot_wrist)]
        colors = ['steelblue', 'mediumseagreen']
        camera_type_names = [
            camera_main_name.replace('CAMERA_', '').replace('_POSE', '').title(),
            camera_wrist_name.replace('CAMERA_', '').replace('_POSE', '').title()
        ]
        camera_labels = [
            f'Main ({camera_type_names[0]})',
            f'Wrist ({camera_type_names[1]})'
        ]
    
    # Create grouped bar chart with Translation and Rotation as x-axis categories
    categories = ['Translation', 'Rotation']
    x = np.arange(len(categories))
    width = 0.35 if not include_secondary else 0.25  # Adjust width based on number of cameras
    
    # Plot translation bars on primary axis
    num_cameras = len(camera_labels)
    for i, (label, color) in enumerate(zip(camera_labels, colors)):
        offset = (i - (num_cameras - 1) / 2) * width
        ax.bar(x[0] + offset, trans_means[i], width, yerr=trans_stds[i], 
               capsize=5, label=label, alpha=0.7, color=color)
    
    ax.set_ylabel('Translation from base pose [m]', color='black')
    ax.set_title('(i) Absolute camera pose variations (mean ± std)')
    ax.set_xticks(x)
    ax.set_xticklabels(categories)
    ax.tick_params(axis='y', labelcolor='black')
    ax.grid(True, linewidth=0.3, alpha=0.4, axis='y')
    
    # Set y-limits for translation based on actual values
    max_trans = max(trans_means[i] + trans_stds[i] for i in range(len(trans_means)))
    ax.set_ylim(0, max_trans * 1.2)
    
    # Create secondary y-axis for rotation
    ax2 = ax.twinx()
    
    # Plot rotation bars on secondary axis
    for i, (label, color) in enumerate(zip(camera_labels, colors)):
        offset = (i - (num_cameras - 1) / 2) * width
        ax2.bar(x[1] + offset, rot_means[i], width, yerr=rot_stds[i], 
                capsize=5, alpha=0.7, color=color)
    
    ax2.set_ylabel('Rotation from base pose [°]', color='black')
    ax2.tick_params(axis='y', labelcolor='black')
    
    # Set y-limits for rotation based on actual values
    max_rot = max(rot_means[i] + rot_stds[i] for i in range(len(rot_means)))
    ax2.set_ylim(0, max_rot * 1.2)
    
    # Add legend only once
    ax.legend(loc='upper left')
    
    return ax


def create_dataset_card(obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, 
                       joint_offsets, gripper_offsets, obj_colors, tgt_colors, 
                       camera_poses_main, camera_poses_secondary, camera_poses_wrist, 
                       xlim, ylim, args, output_file, 
                       dataset_name, num_episodes, has_distractors, metadata, include_secondary_cam=True):
    """Create a 3x3 grid dataset card with all visualizations."""
    # Create figure with 3x3 grid
    fig = plt.figure(figsize=(18, 15), dpi=300)
    
    # Add main title and subtitle
    num_cameras = 3 if include_secondary_cam else 2
    distractor_text = " - with distractor items" if has_distractors else " - no distractor items"
    fig.suptitle(f'Dataset Card: {dataset_name}', fontsize=24, fontweight='bold', y=0.995)
    fig.text(0.5, 0.95, f'{num_episodes:,} episodes{distractor_text} - {num_cameras} cameras', ha='center', fontsize=16, style='italic')
    
    # Row 1: Object positions | Target positions | Connections
    ax1 = plt.subplot(3, 3, 1)
    create_scatter_plot_enhanced(obj_pts, obj_yaws, table_heights, 
                                 "(a) Object positions (with yaw & table height)", ax1, xlim, ylim)
    
    ax2 = plt.subplot(3, 3, 2)
    create_scatter_plot_enhanced(tgt_pts, tgt_yaws, table_heights, 
                                 "(b) Target positions (with yaw & table height)", ax2, xlim, ylim)
    
    ax3 = plt.subplot(3, 3, 3)
    create_line_plot(lines, "(c) Object→Target connections", ax3, xlim, ylim)
    
    # Row 2: X/Y distributions | Travel distances | Travel directions
    ax4 = plt.subplot(3, 3, 4)
    create_coordinate_histogram(obj_pts, tgt_pts, args, ax4)
    
    ax5 = plt.subplot(3, 3, 5)
    create_distance_histogram(obj_pts, tgt_pts, args, ax5)
    
    ax6 = plt.subplot(3, 3, 6, polar=True)
    create_direction_plot(obj_pts, tgt_pts, args, ax6)
    
    # Row 3: Joint/Gripper offsets | Color distribution | Camera pose variations
    ax7 = plt.subplot(3, 3, 7)
    create_joint_offset_plot(joint_offsets, gripper_offsets, ax7)
    
    ax8 = plt.subplot(3, 3, 8)
    create_color_distribution_plot(obj_colors, tgt_colors, ax8)
    
    ax9 = plt.subplot(3, 3, 9)
    create_camera_pose_variation_plot(camera_poses_main, camera_poses_secondary, 
                                     camera_poses_wrist, metadata, ax9, include_secondary_cam)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(output_file, bbox_inches='tight', format='pdf')
    plt.close(fig)
    print(f"[OK] Saved dataset card: {output_file}")


def main():
    ap = argparse.ArgumentParser(description="Generate dataset card visualization from YAML benchmark file.")
    ap.add_argument("--in", dest="infile", required=True, help="Input YAML filepath.")
    ap.add_argument("--dataset-name", dest="dataset_name", required=True, help="Name of the dataset (used in title).")
    ap.add_argument("--out", dest="outfile", help="Output filepath for dataset card. If not specified, saves to same directory as input.")
    ap.add_argument("--pad", type=float, default=0.05, help="Relative padding for inferred square bounds (default: 0.05).")
    ap.add_argument("--bins", type=int, default=30, help="Histogram bins for coordinate and distance distributions (default: 30).")
    ap.add_argument("--angle-bins", type=int, default=36, help="Bins for direction histogram (default: 36, i.e., 10° bins).")
    ap.add_argument("--show", action="store_true", help="Display plot interactively in addition to saving.")
    ap.add_argument("--third-cam", action="store_true", help="Include secondary camera in visualization. If not set, only main and wrist cameras are shown.")
    args = ap.parse_args()

    # Load positions and additional data
    (obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, 
     gripper_offsets, obj_colors, tgt_colors, camera_poses_main, camera_poses_secondary, 
     camera_poses_wrist, metadata) = load_positions(args.infile)

    # Get number of episodes and distractor info
    num_episodes = len(obj_pts)
    has_distractors = metadata.get('distractors', False)

    # Infer coordinate limits
    all_pts = obj_pts + tgt_pts
    xlim = ylim = infer_dim(all_pts, pad=args.pad)

    # Determine output file
    if args.outfile:
        output_file = args.outfile
    else:
        base = os.path.splitext(os.path.basename(args.infile))[0]
        out_dir = os.path.dirname(os.path.abspath(args.infile)) or "."
        output_file = os.path.join(out_dir, f"{base}_dataset_card.pdf")

    # Generate dataset card
    create_dataset_card(obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, 
                       joint_offsets, gripper_offsets, obj_colors, tgt_colors,
                       camera_poses_main, camera_poses_secondary, camera_poses_wrist,
                       xlim, ylim, args, output_file,
                       args.dataset_name, num_episodes, has_distractors, metadata, args.third_cam)

    # Show interactively if requested
    if args.show:
        fig = plt.figure(figsize=(18, 15), dpi=100)  # Lower DPI for display
        
        # Add main title and subtitle
        num_cameras = 3 if args.third_cam else 2
        distractor_text = " - with distractor items" if has_distractors else " - no distractor items"
        fig.suptitle(f'Dataset Card for: {args.dataset_name}', fontsize=24, fontweight='bold', y=0.995)
        fig.text(0.5, 0.95, f'{num_episodes:,} episodes{distractor_text} - {num_cameras} cameras', ha='center', fontsize=16, style='italic')
        
        # Row 1
        ax1 = plt.subplot(3, 3, 1)
        create_scatter_plot_enhanced(obj_pts, obj_yaws, table_heights, 
                                     "(a) Object positions (with yaw & table height)", ax1, xlim, ylim)
        
        ax2 = plt.subplot(3, 3, 2)
        create_scatter_plot_enhanced(tgt_pts, tgt_yaws, table_heights, 
                                     "(b) Target positions (with yaw & table height)", ax2, xlim, ylim)
        
        ax3 = plt.subplot(3, 3, 3)
        create_line_plot(lines, "(c) Object→Target connections", ax3, xlim, ylim)
        
        # Row 2
        ax4 = plt.subplot(3, 3, 4)
        create_coordinate_histogram(obj_pts, tgt_pts, args, ax4)
        
        ax5 = plt.subplot(3, 3, 5)
        create_distance_histogram(obj_pts, tgt_pts, args, ax5)
        
        ax6 = plt.subplot(3, 3, 6, polar=True)
        create_direction_plot(obj_pts, tgt_pts, args, ax6)
        
        # Row 3
        ax7 = plt.subplot(3, 3, 7)
        create_joint_offset_plot(joint_offsets, gripper_offsets, ax7)
        
        ax8 = plt.subplot(3, 3, 8)
        create_color_distribution_plot(obj_colors, tgt_colors, ax8)
        
        ax9 = plt.subplot(3, 3, 9)
        create_camera_pose_variation_plot(camera_poses_main, camera_poses_secondary, 
                                         camera_poses_wrist, metadata, ax9, args.third_cam)
        
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()


if __name__ == "__main__":
    main()
