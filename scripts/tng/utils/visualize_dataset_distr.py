
"""
Dataset Distribution Visualization Tool

This script generates comprehensive visualizations of object/target distributions 
from robotics benchmark YAML files. It provides two main modes:

1. Combined Analysis (default): Creates a single figure with 6 subplots showing:
   - Object position scatter plot
   - Target position scatter plot  
   - Object→Target connection lines
   - Combined X/Y coordinate distributions (side-by-side histogram)
   - Travel distance distribution
   - Unweighted travel direction distribution (polar plot)

2. Separate Plots (--separate): Generates the same 6 analyses as individual files

Features:
- Automatic coordinate range inference with configurable padding
- Side-by-side histograms for easy comparison of coordinate distributions
- Polar plots for directional analysis
- Interactive display option (--show)
- Consistent styling and formatting across all plots

Usage:
    python visualize_dataset_distr.py --in benchmark.yaml [options]
    
Options:
    --separate      Generate individual plot files instead of combined analysis
    --show         Display plots interactively in addition to saving
    --bins N       Number of histogram bins (default: 60)
    --angle-bins N Number of direction histogram bins (default: 72)
    --pad RATIO    Padding for coordinate bounds (default: 0.05)
"""

import argparse
import os
import math
import yaml
import matplotlib.pyplot as plt
import numpy as np


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
        
        obj_pts.append((ox, oy))
        tgt_pts.append((tx, ty))
        lines.append(((ox, oy), (tx, ty)))
        obj_yaws.append(obj_yaw)
        tgt_yaws.append(tgt_yaw)
        table_heights.append(table_height)
        joint_offsets.append(robot_joints)
        gripper_offsets.append(gripper_offset)

    return obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, gripper_offsets, metadata


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


def create_scatter_plot(points, title, ax=None, xlim=None, ylim=None):
    """Create a scatter plot of points."""
    if ax is None:
        fig, ax = plt.subplots()
        standalone = True
    else:
        standalone = False
    
    if points:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        ax.scatter(xs, ys, s=12)
    
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    
    if xlim and ylim:
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        add_square_boundary(ax, xlim, ylim)
    
    ax.grid(True, linewidth=0.3, alpha=0.4)
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax


def create_scatter_plot_enhanced(points, yaws, table_heights, title, ax=None, xlim=None, ylim=None):
    """Create an enhanced scatter plot with yaw angles and table height color grading."""
    import numpy as np
    
    if ax is None:
        fig, ax = plt.subplots()
        standalone = True
    else:
        standalone = False
    
    if points and yaws and table_heights:
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        
        # Create color mapping based on table heights
        heights = np.array(table_heights)
        
        # Create scatter plot with color grading
        scatter = ax.scatter(xs, ys, c=heights, s=40, cmap='viridis', alpha=0.7)
        
        # Add colorbar for table heights
        if standalone or True:  # Always add colorbar for now
            plt.colorbar(scatter, ax=ax, label='Table Height Offset [m]')
        
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
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax


def create_dataset_info_table(yaml_path, obj_pts, tgt_pts, table_heights, joint_offsets, gripper_offsets, metadata=None, ax=None):
    """Create a table with dataset information, statistics, and metadata."""
    import numpy as np
    import os
    
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 8))
        standalone = True
    else:
        standalone = False
    
    # Calculate statistics
    num_episodes = len(obj_pts)
    
    # Create table data - start with total episodes
    table_data = [
        ['Total Episodes', f'{num_episodes:,}'],
    ]
    
    # Add metadata if available
    if metadata:
        # Add all metadata fields
        for key, value in metadata.items():
            # Format the key (replace underscores with spaces, capitalize)
            formatted_key = key.replace('_', ' ').title()
            
            # Format the value based on type
            if isinstance(value, bool):
                formatted_value = 'Yes' if value else 'No'
            elif isinstance(value, list):
                # Handle lists (like required_metrics)
                if len(value) <= 3:
                    formatted_value = ', '.join(str(v) for v in value)
                else:
                    formatted_value = f"{', '.join(str(v) for v in value[:3])}, ... ({len(value)} total)"
            elif value is None:
                formatted_value = 'None'
            else:
                formatted_value = str(value)
            
            table_data.append([formatted_key, formatted_value])
    
    # Remove axis ticks and spines
    ax.axis('off')
    
    # Create table
    table = ax.table(cellText=table_data,
                     cellLoc='left',
                     loc='center',
                     colWidths=[0.50, 0.40])  # Keep left column at 50%, reduce right column to 30%
    
    # Style the table
    table.auto_set_font_size(False)
    table.set_fontsize(7)  # Keep font size at 7
    table.scale(1, 1.3)  # Reduced cell height for more compact table
    
    # Style data rows with alternating colors
    for i in range(len(table_data)):
        if i % 2 == 0:
            table[(i, 0)].set_facecolor('#F5F5F5')
            table[(i, 1)].set_facecolor('#F5F5F5')
        else:
            table[(i, 0)].set_facecolor('white')
            table[(i, 1)].set_facecolor('white')
        
        # Make the first column (labels) bold
        table[(i, 0)].set_text_props(weight='bold')
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax


def create_joint_offset_plot(joint_offsets, gripper_offsets, ax=None):
    """Create a bar plot showing mean and standard deviation of joint and gripper offsets."""
    import numpy as np
    
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 6))
        standalone = True
    else:
        standalone = False
    
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
            "shoulder_pan_joint",
            "shoulder_lift_joint", 
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        
        # Create bar plot for joints only on primary axis
        joint_x_pos = np.arange(len(joint_names))
        joint_bars = ax.bar(joint_x_pos, joint_means, yerr=joint_stds, capsize=5, alpha=0.7, color='darkblue')
        
        ax.set_xlabel('Joint/Gripper')
        ax.set_ylabel('Joint Offset [degrees]', color='darkblue')
        ax.set_title('Robot Joint and Gripper Starting Position Offsets (Mean ± Std)')
        ax.set_xticks(np.arange(len(joint_names) + 1))  # Add space for gripper
        ax.set_xticklabels(joint_names + ['Gripper'], rotation=45)
        ax.grid(True, linewidth=0.3, alpha=0.4)
        ax.tick_params(axis='y', labelcolor='darkblue')
        
        # Add zero line for joints
        ax.axhline(y=0, color='darkblue', linestyle='-', linewidth=0.5, alpha=0.7)
        
        # Create secondary y-axis for gripper
        ax2 = ax.twinx()
        gripper_x_pos = len(joint_names)  # Position at the end
        gripper_bar = ax2.bar(gripper_x_pos, gripper_mean, yerr=gripper_std, capsize=5, alpha=0.7, color='darkorange')
        
        ax2.set_ylabel('Gripper Offset [mm]', color='darkorange')
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
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax


def create_line_plot(lines, title, ax=None, xlim=None, ylim=None):
    """Create a line plot showing connections."""
    if ax is None:
        fig, ax = plt.subplots()
        standalone = True
    else:
        standalone = False
    
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
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax


def create_coordinate_histogram(obj_pts, tgt_pts, args, ax=None):
    """Create combined X/Y coordinate distribution histogram."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 6))
        standalone = True
    else:
        standalone = False
    
    data_sets, labels, colors, coord_range = setup_coordinate_histogram_data(obj_pts, tgt_pts)
    
    if data_sets:
        ax.hist(data_sets, bins=args.bins, range=coord_range, 
                label=labels, color=colors, alpha=0.7, 
                histtype='bar', rwidth=1.0)
        ax.legend()
    
    ax.set_title("Combined X/Y coordinate distributions")
    ax.set_xlabel("position [m]")
    ax.set_ylabel("count")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax


def create_distance_histogram(obj_pts, tgt_pts, args, ax=None):
    """Create travel distance distribution histogram."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(8, 6))
        standalone = True
    else:
        standalone = False
    
    distances, _ = calculate_travel_metrics(obj_pts, tgt_pts)
    
    if distances:
        ax.hist(distances, bins=args.bins, alpha=0.7, color='green')
    
    ax.set_title("Travel distance distribution")
    ax.set_xlabel("distance [m]")
    ax.set_ylabel("count")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax


def create_direction_plot(obj_pts, tgt_pts, args, ax=None):
    """Create unweighted direction distribution polar plot."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))
        standalone = True
    else:
        standalone = False
    
    _, angles_rad = calculate_travel_metrics(obj_pts, tgt_pts)
    
    if angles_rad:
        ax.hist(angles_rad, bins=args.angle_bins, range=(-math.pi, math.pi))
    
    ax.set_title("Travel direction distribution", va="bottom")
    setup_polar_plot(ax)
    
    if standalone:
        plt.tight_layout()
        return fig
    return ax

def save_scatter_plot(points, title, outfile, xlim=None, ylim=None):
    """Save scatter plot to file."""
    fig, ax = plt.subplots()
    create_scatter_plot(points, title, ax, xlim, ylim)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)


def save_line_plot(lines, title, outfile, xlim=None, ylim=None):
    """Save line plot to file."""
    fig, ax = plt.subplots()
    create_line_plot(lines, title, ax, xlim, ylim)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)


def save_coordinate_histogram(obj_pts, tgt_pts, args, outfile):
    """Save coordinate histogram to file."""
    fig, ax = plt.subplots(figsize=(8, 6))
    create_coordinate_histogram(obj_pts, tgt_pts, args, ax)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {outfile}")


def save_distance_histogram(obj_pts, tgt_pts, args, outfile):
    """Save distance histogram to file."""
    fig, ax = plt.subplots(figsize=(8, 6))
    create_distance_histogram(obj_pts, tgt_pts, args, ax)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {outfile}")


def save_direction_plot(obj_pts, tgt_pts, args, outfile):
    """Save direction plot to file."""
    fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))
    create_direction_plot(obj_pts, tgt_pts, args, ax)
    fig.savefig(outfile, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {outfile}")


# Legacy functions for backward compatibility (not currently used)
def xy_hist_figure(points, title_prefix, outfile, bins=60):
    """Two stacked histograms: X distribution (top), Y distribution (bottom)."""
    if not points:
        print(f"[WARN] No points found for {title_prefix}; skipping plot.")
        return

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    # symmetric limits so X/Y are comparable
    max_abs = max(
        max(abs(min(xs)), abs(max(xs))),
        max(abs(min(ys)), abs(max(ys)))
    )
    xr = (-max_abs, max_abs)
    yr = (-max_abs, max_abs)

    fig = plt.figure(figsize=(7, 6), dpi=150)

    ax1 = plt.subplot(2, 1, 1)
    ax1.hist(xs, bins=bins, range=xr)
    ax1.set_title(f"{title_prefix} — X distribution")
    ax1.set_xlabel("x [m]")
    ax1.set_ylabel("count")
    ax1.grid(True, linewidth=0.3, alpha=0.4)

    ax2 = plt.subplot(2, 1, 2)
    ax2.hist(ys, bins=bins, range=yr)
    ax2.set_title(f"{title_prefix} — Y distribution")
    ax2.set_xlabel("y [m]")
    ax2.set_ylabel("count")
    ax2.grid(True, linewidth=0.3, alpha=0.4)

    plt.tight_layout()
    plt.savefig(outfile, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {outfile}")


def polar_direction_hist(obj_pts, tgt_pts, outfile, angle_bins=72, density=True):
    """
    Polar histogram of travel direction (object -> target), 0°..360°.
    Bars are weighted by travel distance. If density=True, the histogram shows
    proportion of total travel distance per bin (area-normalized).
    """
    if not obj_pts or not tgt_pts:
        print("[WARN] Missing positions for polar histogram; skipping.")
        return

    angles_rad, distances = compute_angles_and_weights(obj_pts, tgt_pts)
    if not angles_rad:
        print("[WARN] No valid (angle, distance) samples; skipping polar histogram.")
        return

    fig = plt.figure(figsize=(6, 6), dpi=150)
    ax = plt.subplot(111, polar=True)

    # range=(-pi, pi) lines up 0 at the positive x-axis; we'll set ticks accordingly
    ax.hist(angles_rad, bins=angle_bins, range=(-math.pi, math.pi),
            weights=distances, density=density)

    ax.set_title("Weighted travel direction distribution", va="bottom")
    setup_polar_plot(ax)

    plt.tight_layout()
    plt.savefig(outfile, dpi=150)
    plt.close(fig)
    print(f"[OK] Saved {outfile}")


def compute_angles_and_weights(obj_pts, tgt_pts):
    """Return angles (radians, in [-pi, pi]) and weights (distances)."""
    angles_rad = []
    distances = []
    for (ox, oy), (tx, ty) in zip(obj_pts, tgt_pts):
        dx = tx - ox
        dy = ty - oy
        dist = math.hypot(dx, dy)
        if dist <= 0.0:
            # zero-distance contributes no weight; skip
            continue
        ang = math.atan2(dy, dx)  # radians in [-pi, pi]
        angles_rad.append(ang)
        distances.append(dist)
    return angles_rad, distances


def create_combined_plot(obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, gripper_offsets, xlim, ylim, args, output_file, metadata=None):
    """Create a combined figure with all plots and dataset info table arranged in a grid."""
    # Determine layout based on whether table is included
    if hasattr(args, 'no_table') and args.no_table:
        # Create a layout without the table (3x3 grid)
        fig = plt.figure(figsize=(18, 12), dpi=150)
        
        # Subplot 1: Enhanced object positions scatter with yaw and table height
        ax1 = plt.subplot(3, 3, 1)
        create_scatter_plot_enhanced(obj_pts, obj_yaws, table_heights, "Object positions (with yaw & table height)", ax1, xlim, ylim)
        
        # Subplot 2: Enhanced target positions scatter with yaw and table height
        ax2 = plt.subplot(3, 3, 2)
        create_scatter_plot_enhanced(tgt_pts, tgt_yaws, table_heights, "Target positions (with yaw & table height)", ax2, xlim, ylim)
        
        # Subplot 3: Connections
        ax3 = plt.subplot(3, 3, 3)
        create_line_plot(lines, "Object→Target connections", ax3, xlim, ylim)
        
        # Subplot 4: Combined X/Y distribution for both objects and targets
        ax4 = plt.subplot(3, 3, 4)
        create_coordinate_histogram(obj_pts, tgt_pts, args, ax4)
        
        # Subplot 5: Travel distance distribution
        ax5 = plt.subplot(3, 3, 5)
        create_distance_histogram(obj_pts, tgt_pts, args, ax5)
        
        # Subplot 6: Unweighted direction histogram (polar)
        ax6 = plt.subplot(3, 3, 6, polar=True)
        create_direction_plot(obj_pts, tgt_pts, args, ax6)
        
        # Subplot 7-8: Joint and gripper offset analysis (spans columns 7-8)
        ax7 = plt.subplot(3, 3, (7, 8))
        create_joint_offset_plot(joint_offsets, gripper_offsets, ax7)
    else:
        # Create a layout with the table (4x3 grid)
        fig = plt.figure(figsize=(18, 16), dpi=150)
        
        # Subplot 1: Dataset information table (moved to first position)
        ax1 = plt.subplot(4, 3, 1)
        create_dataset_info_table(args.infile, obj_pts, tgt_pts, table_heights, joint_offsets, gripper_offsets, metadata, ax1)
        
        # Subplot 2: Enhanced object positions scatter with yaw and table height
        ax2 = plt.subplot(4, 3, 2)
        create_scatter_plot_enhanced(obj_pts, obj_yaws, table_heights, "Object positions (with yaw & table height)", ax2, xlim, ylim)
        
        # Subplot 3: Enhanced target positions scatter with yaw and table height
        ax3 = plt.subplot(4, 3, 3)
        create_scatter_plot_enhanced(tgt_pts, tgt_yaws, table_heights, "Target positions (with yaw & table height)", ax3, xlim, ylim)
        
        # Subplot 4: Connections
        ax4 = plt.subplot(4, 3, 4)
        create_line_plot(lines, "Object→Target connections", ax4, xlim, ylim)
        
        # Subplot 5: Combined X/Y distribution for both objects and targets
        ax5 = plt.subplot(4, 3, 5)
        create_coordinate_histogram(obj_pts, tgt_pts, args, ax5)
        
        # Subplot 6: Travel distance distribution
        ax6 = plt.subplot(4, 3, 6)
        create_distance_histogram(obj_pts, tgt_pts, args, ax6)
        
        # Subplot 7: Unweighted direction histogram (polar)
        ax7 = plt.subplot(4, 3, 7, polar=True)
        create_direction_plot(obj_pts, tgt_pts, args, ax7)
        
        # Subplot 8-9: Joint and gripper offset analysis (spans columns 8-9)
        ax8 = plt.subplot(4, 3, (8, 9))
        create_joint_offset_plot(joint_offsets, gripper_offsets, ax8)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] Saved combined plot: {output_file}")


def create_combined_plot_legacy(obj_pts, tgt_pts, lines, xlim, ylim, args, output_file):
    """Create a combined figure with the original 6 plots arranged in a grid (legacy version)."""
    # Create a large figure with subplots arranged in 3 rows, 3 columns
    fig = plt.figure(figsize=(15, 12), dpi=150)
    
    # Subplot 1: Object positions scatter
    ax1 = plt.subplot(3, 3, 1)
    create_scatter_plot(obj_pts, "Object position distribution", ax1, xlim, ylim)
    
    # Subplot 2: Target positions scatter
    ax2 = plt.subplot(3, 3, 2)
    create_scatter_plot(tgt_pts, "Target position distribution", ax2, xlim, ylim)
    
    # Subplot 3: Connections
    ax3 = plt.subplot(3, 3, 3)
    create_line_plot(lines, "Object→Target connections", ax3, xlim, ylim)
    
    # Subplot 4: Combined X/Y distribution for both objects and targets
    ax4 = plt.subplot(3, 3, 4)
    create_coordinate_histogram(obj_pts, tgt_pts, args, ax4)
    
    # Subplot 5: Travel distance distribution
    ax5 = plt.subplot(3, 3, 5)
    create_distance_histogram(obj_pts, tgt_pts, args, ax5)
    
    # Subplot 6: Unweighted direction histogram (polar)
    ax6 = plt.subplot(3, 3, 6, polar=True)
    create_direction_plot(obj_pts, tgt_pts, args, ax6)
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] Saved combined plot: {output_file}")



def save_dataset_info_table(yaml_path, obj_pts, tgt_pts, table_heights, joint_offsets, gripper_offsets, metadata, outfile):
    """Save dataset info table to file."""
    fig, ax = plt.subplots(figsize=(10, 8))
    create_dataset_info_table(yaml_path, obj_pts, tgt_pts, table_heights, joint_offsets, gripper_offsets, metadata, ax)
    fig.savefig(outfile, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] Saved {outfile}")


def save_enhanced_scatter_plot(points, yaws, table_heights, title, outfile, xlim=None, ylim=None):
    """Save enhanced scatter plot to file."""
    fig, ax = plt.subplots()
    create_scatter_plot_enhanced(points, yaws, table_heights, title, ax, xlim, ylim)
    fig.savefig(outfile, dpi=150, bbox_inches='tight')
    plt.close(fig)


def save_joint_offset_plot(joint_offsets, gripper_offsets, outfile):
    """Save joint offset plot to file."""
    fig, ax = plt.subplots(figsize=(10, 6))
    create_joint_offset_plot(joint_offsets, gripper_offsets, ax)
    fig.savefig(outfile, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] Saved {outfile}")



def main():
    ap = argparse.ArgumentParser(description="Plot object/target distributions, connections, and direction histograms from YAML.")
    ap.add_argument("--in", dest="infile", required=True, help="Input YAML filepath.")
    ap.add_argument("--out-dir", dest="out_dir", type=str, help="Output directory for plots. If not specified, uses the same directory as input file.")
    ap.add_argument("--pad", type=float, default=0.05, help="Relative padding for inferred square bounds.")
    ap.add_argument("--bins", type=int, default=30, help="Histogram bins for X/Y distributions.")
    ap.add_argument("--angle-bins", type=int, default=36, help="Bins for direction histogram (72 -> 5° bins).")
    ap.add_argument("--density", action="store_true", help="Normalize polar histogram to proportion of total distance.")
    ap.add_argument("--show", action="store_true", help="Also show plots interactively.")
    ap.add_argument("--separate", action="store_true", help="Save individual plots separately matching the combined analysis.")
    ap.add_argument("--legacy", action="store_true", help="Use legacy plotting without enhanced features (for backward compatibility).")
    ap.add_argument("--no-table", action="store_true", help="Disable the metadata table in the combined plot.")
    args = ap.parse_args()

    # Load positions and additional data
    data_tuple = load_positions(args.infile)
    
    # Check if we have the enhanced data format (9 values) or legacy format (3 values)
    if len(data_tuple) == 9:
        obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, gripper_offsets, metadata = data_tuple
        has_enhanced_data = True
    elif len(data_tuple) == 8:
        # Legacy enhanced data format (without metadata)
        obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, gripper_offsets = data_tuple
        metadata = {}
        has_enhanced_data = True
    else:
        # Legacy format compatibility
        obj_pts, tgt_pts, lines = data_tuple[:3]
        obj_yaws = [0.0] * len(obj_pts)
        tgt_yaws = [0.0] * len(tgt_pts) 
        table_heights = [0.0] * len(obj_pts)
        joint_offsets = [[0.0] * 6] * len(obj_pts)
        gripper_offsets = [0.0] * len(obj_pts)
        metadata = {}
        has_enhanced_data = False

    all_pts = obj_pts + tgt_pts
    xlim = ylim = infer_dim(all_pts, pad=args.pad)

    base = os.path.splitext(os.path.basename(args.infile))[0]
    # Use specified output directory or default to input file's directory
    if args.out_dir:
        out_dir = os.path.abspath(args.out_dir)
        # Create output directory if it doesn't exist
        os.makedirs(out_dir, exist_ok=True)
    else:
        out_dir = os.path.dirname(os.path.abspath(args.infile)) or "."

    if args.separate:
        # Save individual plots
        if has_enhanced_data and not args.legacy:
            # Enhanced separate plots
            out1 = os.path.join(out_dir, f"{base}_objects_enhanced.png")
            out2 = os.path.join(out_dir, f"{base}_targets_enhanced.png")
            out3 = os.path.join(out_dir, f"{base}_connections.png")
            out4 = os.path.join(out_dir, f"{base}_combined_xy_hist.png")
            out5 = os.path.join(out_dir, f"{base}_travel_distance_hist.png")
            out6 = os.path.join(out_dir, f"{base}_direction_unweighted_polar.png")
            out7 = os.path.join(out_dir, f"{base}_joint_offsets.png")
            out8 = os.path.join(out_dir, f"{base}_dataset_info.png")

            save_enhanced_scatter_plot(obj_pts, obj_yaws, table_heights, "Object position distribution", out1, xlim, ylim)
            save_enhanced_scatter_plot(tgt_pts, tgt_yaws, table_heights, "Target position distribution", out2, xlim, ylim)
            save_line_plot(lines, "Object→Target connections", out3, xlim, ylim)
            save_coordinate_histogram(obj_pts, tgt_pts, args, out4)
            save_distance_histogram(obj_pts, tgt_pts, args, out5)
            save_direction_plot(obj_pts, tgt_pts, args, out6)
            save_joint_offset_plot(joint_offsets, gripper_offsets, out7)
            save_dataset_info_table(args.infile, obj_pts, tgt_pts, table_heights, joint_offsets, gripper_offsets, metadata, out8)

            print("Saved enhanced plots:")
            print("  ", out1)
            print("  ", out2)
            print("  ", out3)
            print("  ", out4)
            print("  ", out5)
            print("  ", out6)
            print("  ", out7)
            print("  ", out8)
        else:
            # Legacy separate plots
            out1 = os.path.join(out_dir, f"{base}_objects.png")
            out2 = os.path.join(out_dir, f"{base}_targets.png")
            out3 = os.path.join(out_dir, f"{base}_connections.png")
            out4 = os.path.join(out_dir, f"{base}_combined_xy_hist.png")
            out5 = os.path.join(out_dir, f"{base}_travel_distance_hist.png")
            out6 = os.path.join(out_dir, f"{base}_direction_unweighted_polar.png")

            save_scatter_plot(obj_pts, "Object position distribution", out1, xlim, ylim)
            save_scatter_plot(tgt_pts, "Target position distribution", out2, xlim, ylim)
            save_line_plot(lines, "Object→Target connections", out3, xlim, ylim)
            save_coordinate_histogram(obj_pts, tgt_pts, args, out4)
            save_distance_histogram(obj_pts, tgt_pts, args, out5)
            save_direction_plot(obj_pts, tgt_pts, args, out6)

            print("Saved legacy plots:")
            print("  ", out1)
            print("  ", out2)
            print("  ", out3)
            print("  ", out4)
            print("  ", out5)
            print("  ", out6)
    else:
        # Save combined plot
        if has_enhanced_data and not args.legacy:
            combined_output = os.path.join(out_dir, f"{base}_combined_analysis_enhanced.png")
            create_combined_plot(obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, gripper_offsets, xlim, ylim, args, combined_output, metadata)
        else:
            combined_output = os.path.join(out_dir, f"{base}_combined_analysis.png")
            create_combined_plot_legacy(obj_pts, tgt_pts, lines, xlim, ylim, args, combined_output)

    if args.show:
        if args.separate:
            if has_enhanced_data and not args.legacy:
                # Show enhanced individual plots
                create_scatter_plot_enhanced(obj_pts, obj_yaws, table_heights, "Object position distribution", xlim=xlim, ylim=ylim)
                create_scatter_plot_enhanced(tgt_pts, tgt_yaws, table_heights, "Target position distribution", xlim=xlim, ylim=ylim)
                create_line_plot(lines, "Object→Target connections", xlim=xlim, ylim=ylim)
                create_coordinate_histogram(obj_pts, tgt_pts, args)
                create_distance_histogram(obj_pts, tgt_pts, args)
                create_direction_plot(obj_pts, tgt_pts, args)
                create_joint_offset_plot(joint_offsets, gripper_offsets)
            else:
                # Show legacy individual plots
                create_scatter_plot(obj_pts, "Object position distribution", xlim=xlim, ylim=ylim)
                create_scatter_plot(tgt_pts, "Target position distribution", xlim=xlim, ylim=ylim)
                create_line_plot(lines, "Object→Target connections", xlim=xlim, ylim=ylim)
                create_coordinate_histogram(obj_pts, tgt_pts, args)
                create_distance_histogram(obj_pts, tgt_pts, args)
                create_direction_plot(obj_pts, tgt_pts, args)
        else:
            # Show combined plot using the same function but with display settings
            if has_enhanced_data and not args.legacy:
                # Determine layout based on whether table is included
                if hasattr(args, 'no_table') and args.no_table:
                    fig = plt.figure(figsize=(18, 12), dpi=100)  # Lower DPI for display
                    
                    ax1 = plt.subplot(3, 3, 1)
                    create_scatter_plot_enhanced(obj_pts, obj_yaws, table_heights, "Object positions (with yaw & table height)", ax1, xlim, ylim)
                    
                    ax2 = plt.subplot(3, 3, 2)
                    create_scatter_plot_enhanced(tgt_pts, tgt_yaws, table_heights, "Target positions (with yaw & table height)", ax2, xlim, ylim)
                    
                    ax3 = plt.subplot(3, 3, 3)
                    create_line_plot(lines, "Object→Target connections", ax3, xlim, ylim)
                    
                    ax4 = plt.subplot(3, 3, 4)
                    create_coordinate_histogram(obj_pts, tgt_pts, args, ax4)
                    
                    ax5 = plt.subplot(3, 3, 5)
                    create_distance_histogram(obj_pts, tgt_pts, args, ax5)
                    
                    ax6 = plt.subplot(3, 3, 6, polar=True)
                    create_direction_plot(obj_pts, tgt_pts, args, ax6)
                    
                    ax7 = plt.subplot(3, 3, (7, 8))
                    create_joint_offset_plot(joint_offsets, gripper_offsets, ax7)
                else:
                    fig = plt.figure(figsize=(18, 16), dpi=100)  # Lower DPI for display
                    
                    # Enhanced combined plot for display
                    ax1 = plt.subplot(4, 3, 1)
                    create_dataset_info_table(args.infile, obj_pts, tgt_pts, table_heights, joint_offsets, gripper_offsets, metadata, ax1)
                    
                    ax2 = plt.subplot(4, 3, 2)
                    create_scatter_plot_enhanced(obj_pts, obj_yaws, table_heights, "Object positions (with yaw & table height)", ax2, xlim, ylim)
                    
                    ax3 = plt.subplot(4, 3, 3)
                    create_scatter_plot_enhanced(tgt_pts, tgt_yaws, table_heights, "Target positions (with yaw & table height)", ax3, xlim, ylim)
                    
                    ax4 = plt.subplot(4, 3, 4)
                    create_line_plot(lines, "Object→Target connections", ax4, xlim, ylim)
                    
                    ax5 = plt.subplot(4, 3, 5)
                    create_coordinate_histogram(obj_pts, tgt_pts, args, ax5)
                    
                    ax6 = plt.subplot(4, 3, 6)
                    create_distance_histogram(obj_pts, tgt_pts, args, ax6)
                    
                    ax7 = plt.subplot(4, 3, 7, polar=True)
                    create_direction_plot(obj_pts, tgt_pts, args, ax7)
                    
                    ax8 = plt.subplot(4, 3, (8, 9))
                    create_joint_offset_plot(joint_offsets, gripper_offsets, ax8)
                
                plt.tight_layout()
            else:
                # Legacy combined plot for display
                fig = plt.figure(figsize=(15, 12), dpi=100)  # Lower DPI for display
                
                ax1 = plt.subplot(3, 3, 1)
                create_scatter_plot(obj_pts, "Object position distribution", ax1, xlim, ylim)
                
                ax2 = plt.subplot(3, 3, 2)
                create_scatter_plot(tgt_pts, "Target position distribution", ax2, xlim, ylim)
                
                ax3 = plt.subplot(3, 3, 3)
                create_line_plot(lines, "Object→Target connections", ax3, xlim, ylim)
                
                ax4 = plt.subplot(3, 3, 4)
                create_coordinate_histogram(obj_pts, tgt_pts, args, ax4)
                
                ax5 = plt.subplot(3, 3, 5)
                create_distance_histogram(obj_pts, tgt_pts, args, ax5)
                
                ax6 = plt.subplot(3, 3, 6, polar=True)
                create_direction_plot(obj_pts, tgt_pts, args, ax6)
                
                plt.tight_layout()

        plt.show()


if __name__ == "__main__":
    main()
