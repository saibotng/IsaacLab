
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
    """Load object and target positions from YAML file."""
    with open(yaml_path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    cases = doc.get("test_cases", [])

    obj_pts = []
    tgt_pts = []
    lines = []  # ( (ox,oy), (tx,ty) )

    for c in cases:
        ox, oy, _ = c["object"]["pos"]
        tx, ty, _ = c["target"]["pos"]
        obj_pts.append((ox, oy))
        tgt_pts.append((tx, ty))
        lines.append(((ox, oy), (tx, ty)))

    return obj_pts, tgt_pts, lines


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


def create_combined_plot(obj_pts, tgt_pts, lines, xlim, ylim, args, output_file):
    """Create a combined figure with all 6 plots arranged in a grid."""
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
    
    # Subplots 7-9: Keep empty or use for additional analysis
    # This gives us a cleaner 2x3 layout for the main plots
    
    plt.tight_layout()
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"[OK] Saved combined plot: {output_file}")



def main():
    ap = argparse.ArgumentParser(description="Plot object/target distributions, connections, and direction histograms from YAML.")
    ap.add_argument("--in", dest="infile", required=True, help="Input YAML filepath.")
    ap.add_argument("--pad", type=float, default=0.05, help="Relative padding for inferred square bounds.")
    ap.add_argument("--bins", type=int, default=30, help="Histogram bins for X/Y distributions.")
    ap.add_argument("--angle-bins", type=int, default=36, help="Bins for direction histogram (72 -> 5° bins).")
    ap.add_argument("--density", action="store_true", help="Normalize polar histogram to proportion of total distance.")
    ap.add_argument("--show", action="store_true", help="Also show plots interactively.")
    ap.add_argument("--separate", action="store_true", help="Save individual plots separately matching the combined analysis.")
    args = ap.parse_args()

    obj_pts, tgt_pts, lines = load_positions(args.infile)
    all_pts = obj_pts + tgt_pts
    xlim = ylim = infer_dim(all_pts, pad=args.pad)

    base = os.path.splitext(os.path.basename(args.infile))[0]
    out_dir = os.path.dirname(os.path.abspath(args.infile)) or "."

    if args.separate:
        # New behavior: save the same 6 plots as in combined analysis, but separately
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

        print("Saved plots:")
        print("  ", out1)
        print("  ", out2)
        print("  ", out3)
        print("  ", out4)
        print("  ", out5)
        print("  ", out6)
    else:
        # New behavior: save combined plot
        combined_output = os.path.join(out_dir, f"{base}_combined_analysis.png")
        create_combined_plot(obj_pts, tgt_pts, lines, xlim, ylim, args, combined_output)

    if args.show:
        if args.separate:
            # Show the same 6 plots that are generated in separate mode
            create_scatter_plot(obj_pts, "Object position distribution", xlim=xlim, ylim=ylim)
            create_scatter_plot(tgt_pts, "Target position distribution", xlim=xlim, ylim=ylim)
            create_line_plot(lines, "Object→Target connections", xlim=xlim, ylim=ylim)
            create_coordinate_histogram(obj_pts, tgt_pts, args)
            create_distance_histogram(obj_pts, tgt_pts, args)
            create_direction_plot(obj_pts, tgt_pts, args)
        else:
            # Show the combined plot using the same function but with display settings
            fig = plt.figure(figsize=(15, 12), dpi=100)  # Lower DPI for display
            
            # Subplot 1: Object positions scatter
            ax1 = plt.subplot(3, 3, 1)
            create_scatter_plot(obj_pts, "Object position distribution", ax1, xlim, ylim)
            
            # Subplot 2: Target positions scatter
            ax2 = plt.subplot(3, 3, 2)
            create_scatter_plot(tgt_pts, "Target position distribution", ax2, xlim, ylim)
            
            # Subplot 3: Connections
            ax3 = plt.subplot(3, 3, 3)
            create_line_plot(lines, "Object→Target connections", ax3, xlim, ylim)
            
            # Subplot 4: Combined X/Y distribution
            ax4 = plt.subplot(3, 3, 4)
            create_coordinate_histogram(obj_pts, tgt_pts, args, ax4)
            
            # Subplot 5: Travel distance distribution
            ax5 = plt.subplot(3, 3, 5)
            create_distance_histogram(obj_pts, tgt_pts, args, ax5)
            
            # Subplot 6: Unweighted direction histogram (polar)
            ax6 = plt.subplot(3, 3, 6, polar=True)
            create_direction_plot(obj_pts, tgt_pts, args, ax6)
            
            plt.tight_layout()

        plt.show()


if __name__ == "__main__":
    main()
