import os
import glob
import json
import yaml
import argparse
import math
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np

def load_dataset_positions(yaml_path):
    """Load object and target positions from dataset YAML file."""
    with open(yaml_path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    cases = doc.get("test_cases", [])
    
    obj_pts = []
    tgt_pts = []
    table_heights = []
    
    for c in cases:
        obj_pos = c.get("object", {}).get("pos", [0, 0, 0])
        tgt_pos = c.get("target", {}).get("pos", [0, 0, 0])
        table_height = c.get("table_offset", {}).get("pos", [0, 0, 0])[2]
        
        obj_pts.append((obj_pos[0], obj_pos[1]))
        tgt_pts.append((tgt_pos[0], tgt_pos[1]))
        table_heights.append(table_height)
    
    return obj_pts, tgt_pts, table_heights

def collect_benchmark_results(experiment_dir, benchmark_name):
    """Collect all benchmark results and organize by case ID."""
    pattern = os.path.join(experiment_dir, f"*{benchmark_name}*", "*_results", "results_*.json")
    result_files = glob.glob(pattern)
    
    print(f"Found {len(result_files)} benchmark result files for pattern: {pattern}")
    
    # Dictionary to store results by case_id: {case_id: [results_from_run1, results_from_run2, ...]}
    case_results = defaultdict(list)
    
    for result_file in result_files:
        print(f"Processing: {result_file}")
        with open(result_file, "r") as f:
            data = json.load(f)
        
        run_name = os.path.basename(os.path.dirname(os.path.dirname(result_file)))
        print(f"  Run name: {run_name}")
        print(f"  Total cases in this run: {len(data.get('cases', []))}")
        
        for case in data.get("cases", []):
            case_id = case.get("id")
            if case_id:
                case_info = {
                    "run": run_name,
                    "object_pos": (case.get("object", {}).get("pos", [0, 0, 0])[:2]),
                    "target_pos": (case.get("target", {}).get("pos", [0, 0, 0])[:2]),
                    "success": case.get("success", False),
                    "metrics": case.get("metrics", {})
                }
                case_results[case_id].append(case_info)
    
    print(f"Collected results for {len(case_results)} unique test cases across {len(result_files)} runs")
    return case_results

def calculate_success_rates_per_position(case_results):
    """Calculate success rates for each unique position based on metrics."""
    # Group by object positions for general success rate
    obj_position_metrics = defaultdict(list)
    # Group by target positions for general success rate
    tgt_position_metrics = defaultdict(list)
    # Store connections for overall success visualization
    connections = []
    
    print(f"Processing {len(case_results)} unique test cases...")
    
    for case_id, results in case_results.items():
        if not results:
            continue
            
        # Get position from first result (should be same across runs)
        obj_pos = tuple(results[0]["object_pos"])
        tgt_pos = tuple(results[0]["target_pos"])
        
        print(f"Case {case_id}: {len(results)} runs at obj_pos {obj_pos}, tgt_pos {tgt_pos}")
        
        # Collect metrics for this position
        obj_lifted_results = []
        overall_success_results = []
        
        for result in results:
            metrics = result["metrics"]
            obj_lifted_results.append(metrics.get("object_lifted", False))
            overall_success_results.append(result["success"])
        
        # Store general success rates for both positions (no conditioning)
        obj_position_metrics[obj_pos] = overall_success_results
        tgt_position_metrics[tgt_pos] = overall_success_results
        
        # Add connection with overall success rate
        success_rate = sum(overall_success_results) / len(overall_success_results) if overall_success_results else 0
        connections.append((obj_pos, tgt_pos, success_rate))
        
        print(f"  Object lifted: {sum(obj_lifted_results)}/{len(obj_lifted_results)} = {sum(obj_lifted_results)/len(obj_lifted_results):.2f}")
        print(f"  Overall success: {sum(overall_success_results)}/{len(overall_success_results)} = {success_rate:.2f}")
    
    print(f"Found {len(obj_position_metrics)} unique object positions")
    print(f"Found {len(tgt_position_metrics)} unique target positions")
    
    return obj_position_metrics, tgt_position_metrics, connections

def create_success_overlay_plot(dataset_positions, dataset_heights, benchmark_positions, 
                               title, ax, xlim=None, ylim=None, metric_name=""):
    """Create plot with dataset positions in grayscale and benchmark positions with success overlay."""
    
    # Plot dataset positions in grayscale
    if dataset_positions and dataset_heights:
        ds_xs = [p[0] for p in dataset_positions]
        ds_ys = [p[1] for p in dataset_positions]
        
        # Use uniform grayscale for dataset (no table height variation)
        scatter_dataset = ax.scatter(ds_xs, ds_ys, s=20, color='gray', 
                                   alpha=0.4, label='Dataset positions')
    
    # Plot benchmark positions with success color grading
    if benchmark_positions:
        bm_positions = list(benchmark_positions.keys())
        success_rates = []
        
        for pos in bm_positions:
            results = benchmark_positions[pos]
            success_rate = sum(results) / len(results) if results else 0
            success_rates.append(success_rate)
        
        if bm_positions:
            bm_xs = [p[0] for p in bm_positions]
            bm_ys = [p[1] for p in bm_positions]
            
            # Use red-green colormap for success rates with square markers
            scatter_benchmark = ax.scatter(bm_xs, bm_ys, c=success_rates, s=60, 
                                         cmap='RdYlGn', vmin=0, vmax=1, 
                                         edgecolors='black', linewidth=0.5,
                                         marker='s', label=f'Benchmark positions')
            
            # Add colorbar with better positioning
            cbar = plt.colorbar(scatter_benchmark, ax=ax, shrink=0.8, pad=0.02)
            cbar.set_label(f'Success Rate', fontsize=8)
            cbar.ax.tick_params(labelsize=7)
    
    ax.set_xlabel("x [m]", fontsize=9)
    ax.set_ylabel("y [m]", fontsize=9)
    ax.set_title(title, fontsize=10, pad=10)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    
    # Position legend above the plot to avoid overlap with colorbar
    ax.legend(fontsize=8, bbox_to_anchor=(0.5, -0.1), loc='upper center', ncol=2)
    ax.tick_params(labelsize=8)
    
    if xlim and ylim:
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)

def create_connection_success_plot(connections, title, ax, xlim=None, ylim=None):
    """Create plot showing object-target connections colored by success rate."""
    
    for obj_pos, tgt_pos, success_rate in connections:
        # Color based on success rate (red=0, green=1)
        color = plt.cm.RdYlGn(success_rate)
        
        xs = [obj_pos[0], tgt_pos[0]]
        ys = [obj_pos[1], tgt_pos[1]]
        
        # Draw connection line
        ax.plot(xs, ys, color=color, linewidth=1.5, alpha=0.7)
        
        # Add subtle directional arrow at target end
        # Calculate arrow direction
        dx = tgt_pos[0] - obj_pos[0]
        dy = tgt_pos[1] - obj_pos[1]
        length = (dx**2 + dy**2)**0.5
        
        if length > 0:  # Avoid division by zero
            # Normalize direction and scale for small arrow
            arrow_scale = 0.01  # Small arrow size
            arrow_dx = (dx / length) * arrow_scale
            arrow_dy = (dy / length) * arrow_scale
            
            # Draw small arrow at target position
            ax.annotate('', xy=tgt_pos, xytext=(tgt_pos[0] - arrow_dx, tgt_pos[1] - arrow_dy),
                       arrowprops=dict(arrowstyle='->', color=color, lw=1, alpha=0.8))
    
    ax.set_xlabel("x [m]", fontsize=9)
    ax.set_ylabel("y [m]", fontsize=9)
    ax.set_title(title, fontsize=10, pad=10)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linewidth=0.3, alpha=0.4)
    ax.tick_params(labelsize=8)
    
    # Add colorbar with better positioning
    sm = plt.cm.ScalarMappable(cmap='RdYlGn', norm=plt.Normalize(vmin=0, vmax=1))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label('Success Rate', fontsize=8)
    cbar.ax.tick_params(labelsize=7)
    
    if xlim and ylim:
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)

def create_success_bar_chart(summary, ax):
    """Create a bar chart with error bars showing success rates, uncertainty, and number of runs."""
    
    # Extract data
    averages = summary.get("averages", {})
    statistics = summary.get("statistics", {})
    
    overall_success = averages.get("success_rate", 0) * 100
    overall_std = statistics.get("success_rate_std", 0) * 100
    metric_rates = averages.get("metric_successes_rates", {})
    metric_stds = statistics.get("metric_successes_rates_std", {})
    num_runs = statistics.get("num_runs", 0)
    
    # Prepare data - metrics first, then overall
    metric_names = [name.replace("_", " ").title() for name in metric_rates.keys()]
    metric_values = [rate * 100 for rate in metric_rates.values()]
    metric_errors = [metric_stds.get(metric, 0) * 100 for metric in metric_rates.keys()]
    
    # Create positions - metrics grouped first, gap, then overall
    n_metrics = len(metric_names)
    metric_positions = list(range(n_metrics))  # 0, 1, 2, 3...
    overall_position = [n_metrics + 1]  # Gap of 1 after metrics
    
    # All labels, values, and errors
    all_labels = metric_names + ["Success Rate"]
    all_positions = metric_positions + overall_position
    all_values = metric_values + [overall_success]
    all_errors = metric_errors + [overall_std]
    
    # Colors
    metric_colors = ['lightblue'] * n_metrics
    overall_color = ['darkblue']
    all_colors = metric_colors + overall_color
    
    # Create bars and error bars separately
    widths = [0.5] * n_metrics + [0.6]  # Smaller bars for metrics
    bars = ax.bar(all_positions, all_values, width=widths, color=all_colors, alpha=0.8)
    
    # Add error bars separately
    ax.errorbar(all_positions, all_values, yerr=all_errors, fmt='none', 
                capsize=4, capthick=1.5, linewidth=1.5, ecolor='black', alpha=0.7)
    
    # Add value labels with uncertainty
    for i, (bar, value, error) in enumerate(zip(bars, all_values, all_errors)):
        height = bar.get_height()
        
        # Main percentage label with uncertainty
        ax.text(bar.get_x() + bar.get_width()/2., height + error + 1,
                f'{value:.1f}±{error:.1f}%', 
                ha='center', va='bottom', fontsize=9, fontweight='bold')
    
    # Add separator line
    if n_metrics > 0:
        separator_x = n_metrics - 0.5 + 0.5  # Between last metric and overall
        ax.axvline(x=separator_x, color='gray', linestyle='--', alpha=0.7, linewidth=1)
    
    # Styling
    ax.set_xticks(all_positions)
    ax.set_xticklabels(all_labels, fontsize=9)
    ax.set_ylabel("Success Rate (%)", fontsize=10)
    ax.set_title("Success Rates with Uncertainty", fontsize=12)
    
    # Set y-limit to accommodate error bars
    max_val_with_error = max([v + e for v, e in zip(all_values, all_errors)]) if all_values else 100
    ax.set_ylim(0, max_val_with_error * 1.15)
    
    ax.grid(True, axis='y', alpha=0.3)
    
    # Rotate labels if needed
    if n_metrics > 3:
        plt.setp(ax.get_xticklabels()[:-1], rotation=45, ha='right')

def infer_coordinate_bounds(obj_pts, tgt_pts, pad=0.05):
    all_points = obj_pts + tgt_pts
    if not all_points:
        return (-1, 1), (-1, 1)
    
    xs = [p[0] for p in all_points]
    ys = [p[1] for p in all_points]
    
    x_range = max(xs) - min(xs)
    y_range = max(ys) - min(ys)
    max_range = max(x_range, y_range)
    
    center_x = (max(xs) + min(xs)) / 2
    center_y = (max(ys) + min(ys)) / 2
    
    half_range = max_range / 2 * (1 + pad)
    
    xlim = (center_x - half_range, center_x + half_range)
    ylim = (center_y - half_range, center_y + half_range)
    
    return xlim, ylim

def create_benchmark_visualization(experiment_dir, benchmark_name):
    """Create comprehensive benchmark visualization dashboard."""
    
    # Collect benchmark results
    case_results = collect_benchmark_results(experiment_dir, benchmark_name)
    
    if not case_results:
        print(f"No benchmark results found for {benchmark_name} in {experiment_dir}")
        return None
    
    # Calculate success rates per position
    obj_pos_metrics, tgt_pos_metrics, connections = calculate_success_rates_per_position(case_results)
    
    # Find dataset YAML file in data_visualization directory
    dataset_yaml = None
    data_viz_dir = os.path.join(experiment_dir, "data_visualization")
    if os.path.exists(data_viz_dir):
        yaml_files = glob.glob(os.path.join(data_viz_dir, "*.yaml"))
        if yaml_files:
            dataset_yaml = yaml_files[0]  # Take the first YAML file found
    
    # Load dataset positions if available
    dataset_obj_pts, dataset_tgt_pts, dataset_heights = [], [], []
    if dataset_yaml and os.path.exists(dataset_yaml):
        print(f"Loading dataset positions from: {dataset_yaml}")
        dataset_obj_pts, dataset_tgt_pts, dataset_heights = load_dataset_positions(dataset_yaml)
    else:
        print("No dataset YAML file found in data_visualization directory")
    
    # Infer coordinate bounds
    all_obj_pts = dataset_obj_pts + list(obj_pos_metrics.keys())
    all_tgt_pts = dataset_tgt_pts + list(tgt_pos_metrics.keys())
    xlim, ylim = infer_coordinate_bounds(all_obj_pts, all_tgt_pts)
    
    # Create visualization dashboard
    fig = plt.figure(figsize=(20, 14))
    
    # Add comprehensive heading for the entire plot collection
    experiment_name = os.path.basename(os.path.normpath(experiment_dir))
    summary = summarize_benchmarks(experiment_dir, benchmark_name)
    num_runs = summary.get("statistics", {}).get("num_runs", 0)
    main_title = f"Experiment: {experiment_name} | Benchmark: {benchmark_name} | Runs: {num_runs}"
    fig.suptitle(main_title, fontsize=16, fontweight='bold', y=0.98)
    
    # Plot 1: Object positions with general success overlay
    ax1 = plt.subplot(2, 3, 1)
    create_success_overlay_plot(dataset_obj_pts, dataset_heights, obj_pos_metrics,
                               "Object Positions: Success Rate", ax1, xlim, ylim, "Success Rate")
    
    # Plot 2: Target positions with general success overlay
    ax2 = plt.subplot(2, 3, 2)
    create_success_overlay_plot(dataset_tgt_pts, dataset_heights, tgt_pos_metrics,
                               "Target Positions: Success Rate", ax2, xlim, ylim, "Success Rate")
    
    # Plot 3: Object-target connections with overall success
    ax3 = plt.subplot(2, 3, 3)
    create_connection_success_plot(connections, "Object→Target Connections: Success Rate", ax3, xlim, ylim)
    
    # Plot 4: Success rate bar chart (spans bottom row)
    ax4 = plt.subplot(2, 1, 2)
    
    # Use the already calculated summary for bar chart
    create_success_bar_chart(summary, ax4)
    
    # Adjust layout to prevent occlusions and accommodate legends below plots
    plt.tight_layout(pad=3.0, h_pad=7.0, w_pad=2.5, rect=[0, 0.05, 1, 0.95])
    
    # Save visualization
    output_path = os.path.join(experiment_dir, f"benchmark_analysis_{benchmark_name}.png")
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Benchmark visualization saved to {output_path}")
    return output_path

def summarize_benchmarks(experiment_dir, benchmark_name):
    # Find all benchmark run directories matching the benchmark name
    pattern = os.path.join(experiment_dir, f"*{benchmark_name}*", "*_results", "results_*.json")
    result_files = glob.glob(pattern)
    summary = {
        "runs": []
    }
    averages = {
        "success_rate": None,
        "metric_successes_rates": {}
    }
    statistics = {
        "success_rate_std": None,
        "metric_successes_rates_std": {},
        "num_runs": 0
    }
    metric_totals = defaultdict(list)
    overall_rates = []

    for result_file in result_files:
        with open(result_file, "r") as f:
            data = json.load(f)
        run_summary = {
            "run": os.path.basename(os.path.dirname(os.path.dirname(result_file))),
            "success_rate": data.get("success_rate"),
            "metric_successes_rates": data.get("metric_successes_rates", {})
        }
        summary["runs"].append(run_summary)
        if run_summary["success_rate"] is not None:
            overall_rates.append(run_summary["success_rate"])
        for metric, rate in run_summary["metric_successes_rates"].items():
            metric_totals[metric].append(rate)

    # Calculate averages and standard deviations
    if overall_rates:
        averages["success_rate"] = sum(overall_rates) / len(overall_rates)
        statistics["success_rate_std"] = np.std(overall_rates) if len(overall_rates) > 1 else 0
        statistics["num_runs"] = len(overall_rates)
    
    for metric, rates in metric_totals.items():
        averages["metric_successes_rates"][metric] = sum(rates) / len(rates)
        statistics["metric_successes_rates_std"][metric] = np.std(rates) if len(rates) > 1 else 0

    # Add averages and statistics sections
    summary["averages"] = averages
    summary["statistics"] = statistics
    return summary

def main():
    parser = argparse.ArgumentParser(description="Summarize benchmark results and create visualizations.")
    parser.add_argument("--benchmark_name", help="Benchmark name (e.g. benchmark_easy_default_cams)")
    parser.add_argument("--experiment_dir", help="Experiment directory (e.g. /home/innovation-hacking/luebbet/models/pipeline/dataset_size_30_10k)")
    parser.add_argument("--output", default="summary", help="Output YAML file")
    parser.add_argument("--visualize", action="store_true", help="Create benchmark visualization dashboard")
    args = parser.parse_args()

    if not args.experiment_dir:
        print("Error: --experiment_dir required")
        return
    if not args.benchmark_name:
        print("Error: --benchmark_name required")
        return

    # Generate summary
    summary = summarize_benchmarks(args.experiment_dir, args.benchmark_name)
    output_path = os.path.join(args.experiment_dir, f"{args.output}_{args.benchmark_name}.yaml")
    with open(output_path, "w") as f:
        yaml.dump(summary, f, sort_keys=False)
    print(f"Summary written to {output_path}")
    
    # Generate visualization if requested
    if args.visualize:
        create_benchmark_visualization(args.experiment_dir, args.benchmark_name)

if __name__ == "__main__":
    main()