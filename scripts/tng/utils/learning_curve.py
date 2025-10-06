#!/usr/bin/env python3

import os
import glob
import json
import argparse
import re
from collections import defaultdict
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

def extract_train_steps_from_path(experiment_dir):
    """Extract training steps from experiment directory name."""
    # Try different patterns to extract training steps
    patterns = [
        r'train_steps_(\d+)k',  # train_steps_100k
        r'steps_(\d+)k',       # steps_100k
        r'(\d+)k_steps',       # 100k_steps
        r'checkpoint-(\d+)',   # checkpoint-10000
        r'step_(\d+)',         # step_10000
        r'(\d+)k',             # 100k (if at end)
    ]
    
    dir_name = os.path.basename(experiment_dir)
    
    for pattern in patterns:
        match = re.search(pattern, dir_name, re.IGNORECASE)
        if match:
            steps = int(match.group(1))
            # If pattern includes 'k', multiply by 1000
            if 'k' in pattern:
                steps *= 1000
            return steps
    
    # If no pattern matches, try to extract any number and assume it's steps
    numbers = re.findall(r'\d+', dir_name)
    if numbers:
        return int(numbers[-1])  # Take the last number found
    
    return None

def summarize_single_experiment(experiment_dir, benchmark_name):
    """Get summary statistics for a single experiment including all individual run results."""
    pattern = os.path.join(experiment_dir, f"*{benchmark_name}*", "*_results", "results_*.json")
    result_files = glob.glob(pattern)
    
    if not result_files:
        return None
    
    metric_totals = defaultdict(list)
    overall_rates = []

    for result_file in result_files:
        with open(result_file, "r") as f:
            data = json.load(f)
        
        success_rate = data.get("success_rate")
        if success_rate is not None:
            overall_rates.append(success_rate)
        
        for metric, rate in data.get("metric_successes_rates", {}).items():
            metric_totals[metric].append(rate)

    # Calculate averages and return individual results for uncertainty analysis
    summary = {
        "overall_success_rates": overall_rates,  # Individual run results
        "metric_rates": {}
    }
    
    if overall_rates:
        summary["overall_success_mean"] = sum(overall_rates) / len(overall_rates)
        summary["overall_success_std"] = np.std(overall_rates) if len(overall_rates) > 1 else 0
    
    for metric, rates in metric_totals.items():
        summary["metric_rates"][metric] = rates  # Individual run results
        summary[f"metric_{metric}_mean"] = sum(rates) / len(rates)
        summary[f"metric_{metric}_std"] = np.std(rates) if len(rates) > 1 else 0

    return summary

def collect_experiments_data(experiment_dirs, benchmark_name):
    """Collect success rate data from multiple experiment directories."""
    experiments_data = []
    
    for exp_dir in experiment_dirs:
        if not os.path.exists(exp_dir):
            print(f"Warning: Directory {exp_dir} does not exist")
            continue
            
        # Extract training steps
        train_steps = extract_train_steps_from_path(exp_dir)
        if train_steps is None:
            print(f"Warning: Could not extract training steps from {exp_dir}")
            continue
        
        # Get benchmark summary
        summary = summarize_single_experiment(exp_dir, benchmark_name)
        
        if not summary:
            print(f"Warning: No benchmark data found for {exp_dir}")
            continue
            
        overall_success_mean = summary.get('overall_success_mean', 0) * 100
        overall_success_std = summary.get('overall_success_std', 0) * 100
        overall_success_rates = [rate * 100 for rate in summary.get('overall_success_rates', [])]
        
        # Extract metric rates (exclude overall success rate)
        metric_rates_mean = {}
        metric_rates_all = {}
        for key, value in summary.items():
            if key.startswith('metric_') and key.endswith('_mean'):
                metric_name = key.replace('metric_', '').replace('_mean', '')
                metric_rates_mean[metric_name] = value * 100
                # Get all individual values for this metric
                metric_rates_all[metric_name] = [rate * 100 for rate in summary['metric_rates'].get(metric_name, [])]
        
        experiments_data.append({
            'experiment_dir': exp_dir,
            'experiment_name': os.path.basename(exp_dir),
            'train_steps': train_steps,
            'overall_success_mean': overall_success_mean,
            'overall_success_std': overall_success_std,
            'overall_success_rates': overall_success_rates,  # All individual run results
            'metric_rates_mean': metric_rates_mean,
            'metric_rates_all': metric_rates_all,  # All individual run results per metric
            'num_runs': len(overall_success_rates)
        })
        
        print(f"Processed {os.path.basename(exp_dir)}: {train_steps} steps, {overall_success_mean:.1f}±{overall_success_std:.1f}% success ({len(overall_success_rates)} runs)")
    
    # Sort by training steps
    experiments_data.sort(key=lambda x: x['train_steps'])
    return experiments_data

def create_summary_table(experiments_data):
    """Create a summary table with success rate and metrics (no num runs row)."""
    import pandas as pd
    
    # Sort experiments by training steps
    experiments_data_sorted = sorted(experiments_data, key=lambda x: x['train_steps'])
    
    # Create column headers (train steps)
    columns = [f"{exp['train_steps']//1000}k Steps" for exp in experiments_data_sorted]
    
    # Collect all unique metrics
    all_metrics = set()
    for exp in experiments_data_sorted:
        all_metrics.update(exp['metric_rates_mean'].keys())
    all_metrics = sorted(list(all_metrics))
    
    # Create table data
    table_data = {}
    
    # Row 1: Overall success rate (mean ± std format)
    success_rates = []
    for exp in experiments_data_sorted:
        mean = exp['overall_success_mean']
        std = exp['overall_success_std']
        success_rates.append(f"{mean:.1f}±{std:.1f}%")
    table_data['Success Rate'] = success_rates
    
    # Following rows: Individual metrics (mean ± std format)
    for metric in all_metrics:
        metric_values = []
        for exp in experiments_data_sorted:
            if metric in exp['metric_rates_mean']:
                mean = exp['metric_rates_mean'][metric]
                # Calculate std for this metric
                individual_vals = exp['metric_rates_all'].get(metric, [])
                std = np.std(individual_vals) if len(individual_vals) > 1 else 0
                metric_values.append(f"{mean:.1f}±{std:.1f}%")
            else:
                metric_values.append("--")
        
        # Clean up metric name for display
        metric_display = metric.replace('_', ' ').title()
        table_data[metric_display] = metric_values
    
    # Create DataFrame (transposed so metrics are rows)
    df = pd.DataFrame(table_data, index=columns).T
    
    # Get number of runs for caption (assuming all experiments have same number of runs)
    num_runs = experiments_data_sorted[0]['num_runs'] if experiments_data_sorted else 0
    
    return df, num_runs


def create_overall_success_plot(experiments_data, benchmark_name, output_path):
    """Create individual plot for overall success rate."""
    plt.rcParams.update({
        "pdf.fonttype": 42, "ps.fonttype": 42, "font.size": 10,
        "axes.labelsize": 11, "axes.titlesize": 11, "legend.fontsize": 9,
        "xtick.labelsize": 9, "ytick.labelsize": 9,
    })
    
    fig, ax = plt.subplots(figsize=(10, 4))
    
    train_steps = [exp['train_steps'] for exp in experiments_data]
    overall_success_mean = [exp['overall_success_mean'] for exp in experiments_data]
    overall_success_std = [exp['overall_success_std'] for exp in experiments_data]
    
    ax.errorbar(train_steps, overall_success_mean, yerr=overall_success_std, 
                fmt='o-', linewidth=1.0, markersize=6, capsize=3, capthick=1,
                color='darkblue', label='Overall Success Rate')
    ax.fill_between(train_steps, 
                    [m - s for m, s in zip(overall_success_mean, overall_success_std)],
                    [m + s for m, s in zip(overall_success_mean, overall_success_std)],
                    alpha=0.2, color='lightblue')
    
    # Add success rate labels
    for i, (steps, success, std, exp) in enumerate(zip(train_steps, overall_success_mean, overall_success_std, experiments_data)):
        ax.annotate(f'{success:.1f}±{std:.1f}% (n={exp["num_runs"]})', 
                    (steps, success), 
                    textcoords="offset points", 
                    xytext=(0, 10), 
                    ha='center', 
                    fontsize=8,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8, edgecolor='none'))
    
    ax.set_xlabel('Training Steps')
    ax.set_ylabel('Success Rate (%)')
    ax.set_title(f'Overall Success Rate: {benchmark_name.replace("_", " ").title()}')
    ax.grid(True, alpha=0.3)
    ax.legend(frameon=False)
    
    # Format x-axis
    if max(train_steps) >= 1000:
        ax.set_xticks(train_steps)
        ax.set_xticklabels([f'{int(s/1000)}k' for s in train_steps])
    
    ax.set_ylim(0, max([m + s for m, s in zip(overall_success_mean, overall_success_std)]) * 1.2)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()


def create_metrics_plot(experiments_data, benchmark_name, output_path):
    """Create individual plot for individual metrics."""
    plt.rcParams.update({
        "pdf.fonttype": 42, "ps.fonttype": 42, "font.size": 10,
        "axes.labelsize": 11, "axes.titlesize": 11, "legend.fontsize": 9,
        "xtick.labelsize": 9, "ytick.labelsize": 9,
    })
    
    fig, ax = plt.subplots(figsize=(10, 4))
    
    train_steps = [exp['train_steps'] for exp in experiments_data]
    
    # Get all unique metrics
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    
    if not all_metrics:
        print("No individual metrics found")
        return
    
    import matplotlib.cm as cm
    colors = cm.get_cmap('tab10')(np.linspace(0, 1, len(all_metrics)))
    
    for i, metric in enumerate(sorted(all_metrics)):
        metric_means = []
        metric_stds = []
        for exp in experiments_data:
            mean_val = exp['metric_rates_mean'].get(metric, 0)
            individual_vals = exp['metric_rates_all'].get(metric, [])
            std_val = np.std(individual_vals) if len(individual_vals) > 1 else 0
            metric_means.append(mean_val)
            metric_stds.append(std_val)
        
        ax.errorbar(train_steps, metric_means, yerr=metric_stds,
                    fmt='o-', linewidth=1.0, markersize=4, capsize=2,
                    color=colors[i], label=metric.replace('_', ' ').title(), alpha=0.8)
    
    ax.set_xlabel('Training Steps')
    ax.set_ylabel('Success Rate (%)')
    ax.set_title(f'Individual Metrics: {benchmark_name.replace("_", " ").title()}')
    ax.grid(True, alpha=0.3)
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', frameon=False)
    
    # Format x-axis
    if max(train_steps) >= 1000:
        ax.set_xticks(train_steps)
        ax.set_xticklabels([f'{int(s/1000)}k' for s in train_steps])
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()


def create_learning_curve_plot(experiments_data, benchmark_name, output_path):
    """Create a learning curve plot showing success rates vs training steps with uncertainty."""
    
    if not experiments_data:
        print("No experiment data to plot")
        return
    
    # Apply tensorboard plot styling
    plt.rcParams.update({
        "pdf.fonttype": 42,      # editable text in Illustrator
        "ps.fonttype": 42,
        "font.size": 10,
        "axes.labelsize": 11,
        "axes.titlesize": 11,
        "legend.fontsize": 9,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
    })
    
    # Create figure with 2 subplots: overall success and individual metrics
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Extract data for plotting
    train_steps = [exp['train_steps'] for exp in experiments_data]
    overall_success_mean = [exp['overall_success_mean'] for exp in experiments_data]
    overall_success_std = [exp['overall_success_std'] for exp in experiments_data]
    
    # Plot 1: Overall success rate with error bars (tensorboard style)
    ax1.errorbar(train_steps, overall_success_mean, yerr=overall_success_std, 
                fmt='o-', linewidth=1.0, markersize=6, capsize=3, capthick=1,
                color='darkblue', label='Overall Success Rate')
    ax1.fill_between(train_steps, 
                    [m - s for m, s in zip(overall_success_mean, overall_success_std)],
                    [m + s for m, s in zip(overall_success_mean, overall_success_std)],
                    alpha=0.2, color='lightblue')
    
    # Add success rate labels to each data point
    for i, (steps, success, std, exp) in enumerate(zip(train_steps, overall_success_mean, overall_success_std, experiments_data)):
        ax1.annotate(f'{success:.1f}±{std:.1f}% (n={exp["num_runs"]})', 
                    (steps, success), 
                    textcoords="offset points", 
                    xytext=(0, 15), 
                    ha='center', 
                    fontsize=9, 
                    fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8, edgecolor='none'))
    
    ax1.set_xlabel('Training Steps')
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title(f'Learning Curve: {benchmark_name.replace("_", " ").title()}')
    ax1.grid(True, alpha=0.3)
    ax1.legend(frameon=False)
    ax1.set_ylim(0, max([m + s for m, s in zip(overall_success_mean, overall_success_std)]) * 1.2 if overall_success_mean else 100)
    
    # Format x-axis to show steps nicely
    if max(train_steps) >= 1000:
        ax1.set_xlabel('Training Steps', fontsize=12)
        ax1.set_xticks(train_steps)
        ax1.set_xticklabels([f'{int(s/1000)}k' for s in train_steps])
    
    # Plot 2: Individual metrics with error bars
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    
    if all_metrics:
        import matplotlib.cm as cm
        colors = cm.get_cmap('tab10')(np.linspace(0, 1, len(all_metrics)))
        
        max_metric_value = 0
        for i, metric in enumerate(sorted(all_metrics)):
            metric_means = []
            metric_stds = []
            for exp in experiments_data:
                mean_val = exp['metric_rates_mean'].get(metric, 0)
                # Calculate std for this metric from individual values
                individual_vals = exp['metric_rates_all'].get(metric, [])
                std_val = np.std(individual_vals) if len(individual_vals) > 1 else 0
                metric_means.append(mean_val)
                metric_stds.append(std_val)
            
            max_metric_value = max(max_metric_value, max([m + s for m, s in zip(metric_means, metric_stds)]) if metric_means else 0)
            
            ax2.errorbar(train_steps, metric_means, yerr=metric_stds,
                        fmt='o-', linewidth=1.0, markersize=4, capsize=2,
                        color=colors[i], label=metric.replace('_', ' ').title(), alpha=0.8)
        
        ax2.set_ylabel('Success Rate (%)')
        ax2.set_title('Individual Metrics Learning Curves')
        ax2.grid(True, alpha=0.3)
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left', frameon=False)
        ax2.set_ylim(0, max_metric_value * 1.1 if max_metric_value > 0 else 100)
        
        # Format x-axis for metrics plot
        if max(train_steps) >= 1000:
            ax2.set_xlabel('Training Steps')
            ax2.set_xticks(train_steps)
            ax2.set_xticklabels([f'{int(s/1000)}k' for s in train_steps])
        else:
            ax2.set_xlabel('Training Steps')
    else:
        # Hide second subplot if no metrics
        ax2.set_visible(False)
        fig.set_size_inches(12, 6)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Learning curve saved to {output_path}")
    return output_path


def create_combined_plot(experiments_data, benchmark_name, summary_table, output_path, num_runs):
    """Create a combined plot with both charts and summary table."""
    plt.rcParams.update({
        "pdf.fonttype": 42, "ps.fonttype": 42, "font.size": 9,
        "axes.labelsize": 10, "axes.titlesize": 11, "legend.fontsize": 8,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
    })
    
    # Create figure with 3 subplots: 2 plots on top, table on bottom
    fig = plt.figure(figsize=(16, 14))
    gs = fig.add_gridspec(3, 2, height_ratios=[2, 2, 1.5], hspace=0.5, wspace=0.3)
    
    ax1 = fig.add_subplot(gs[0, :])  # Overall success plot (spans both columns)
    ax2 = fig.add_subplot(gs[1, :])  # Metrics plot (spans both columns)
    ax3 = fig.add_subplot(gs[2, :])  # Table (spans both columns)
    
    train_steps = [exp['train_steps'] for exp in experiments_data]
    overall_success_mean = [exp['overall_success_mean'] for exp in experiments_data]
    overall_success_std = [exp['overall_success_std'] for exp in experiments_data]
    
    # Plot 1: Overall success rate
    ax1.errorbar(train_steps, overall_success_mean, yerr=overall_success_std, 
                fmt='o-', linewidth=1.0, markersize=6, capsize=3, capthick=1,
                color='darkblue', label='Overall Success Rate')
    ax1.fill_between(train_steps, 
                    [m - s for m, s in zip(overall_success_mean, overall_success_std)],
                    [m + s for m, s in zip(overall_success_mean, overall_success_std)],
                    alpha=0.2, color='lightblue')
    
    ax1.set_xlabel('Training Steps')
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title(f'Overall Success Rate: {benchmark_name.replace("_", " ").title()}')
    ax1.grid(True, alpha=0.3)
    ax1.legend(frameon=False)
    
    # Format x-axis
    if max(train_steps) >= 1000:
        ax1.set_xticks(train_steps)
        ax1.set_xticklabels([f'{int(s/1000)}k' for s in train_steps])
    
    # Plot 2: Individual metrics
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    
    if all_metrics:
        import matplotlib.cm as cm
        colors = cm.get_cmap('tab10')(np.linspace(0, 1, len(all_metrics)))
        
        for i, metric in enumerate(sorted(all_metrics)):
            metric_means = []
            metric_stds = []
            for exp in experiments_data:
                mean_val = exp['metric_rates_mean'].get(metric, 0)
                individual_vals = exp['metric_rates_all'].get(metric, [])
                std_val = np.std(individual_vals) if len(individual_vals) > 1 else 0
                metric_means.append(mean_val)
                metric_stds.append(std_val)
            
            ax2.errorbar(train_steps, metric_means, yerr=metric_stds,
                        fmt='o-', linewidth=1.0, markersize=4, capsize=2,
                        color=colors[i], label=metric.replace('_', ' ').title(), alpha=0.8)
        
        ax2.set_xlabel('Training Steps')
        ax2.set_ylabel('Success Rate (%)')
        ax2.set_title(f'Individual Metrics: {benchmark_name.replace("_", " ").title()}')
        ax2.grid(True, alpha=0.3)
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left', frameon=False)
        
        # Format x-axis
        if max(train_steps) >= 1000:
            ax2.set_xticks(train_steps)
            ax2.set_xticklabels([f'{int(s/1000)}k' for s in train_steps])
    else:
        ax2.set_visible(False)
    
    # Plot 3: Summary table (normal format with first column as "Metrics")
    ax3.axis('tight')
    ax3.axis('off')
    
    # Prepare table with "Metrics" as first column header
    col_labels = ["Metrics"] + summary_table.columns.tolist()
    row_labels = summary_table.index.tolist()
    
    # Create table data: each row has metric name + values
    table_data = []
    for idx, row_name in enumerate(row_labels):
        row_data = [row_name] + summary_table.iloc[idx].tolist()
        table_data.append(row_data)
    
    # Create the table (normal format, no row labels since we include them in data)
    table_plot = ax3.table(cellText=table_data,
                          colLabels=col_labels,
                          cellLoc='center',
                          loc='center')
    
    # Style the table with better spacing
    table_plot.auto_set_font_size(False)
    table_plot.set_fontsize(9)
    table_plot.scale(1.2, 2.0)  # Increased width scaling
    
    num_cols = len(col_labels)
    num_rows = len(table_data)
    
    # Color column headers (top row)
    for i in range(num_cols):
        table_plot[(0, i)].set_facecolor('#E6E6E6')
        table_plot[(0, i)].set_text_props(weight='bold', fontsize=9)
    
    # Color first column (metrics names)
    for j in range(1, num_rows + 1):  # Skip header row
        table_plot[(j, 0)].set_facecolor('#F0F0F0')
        table_plot[(j, 0)].set_text_props(weight='bold', fontsize=9)
    
    # Adjust cell properties and make first column wider
    for i in range(num_cols):
        for j in range(num_rows + 1):  # +1 for header
            cell = table_plot[(j, i)]
            cell.set_text_props(wrap=True, fontsize=9)
            cell.PAD = 0.02  # Reduce padding
            
            # Make first column (metrics names) wider
            if i == 0:
                cell.set_width(0.25)  # Increase width of first column
            else:
                cell.set_width(0.15)  # Standard width for data columns
    
    # Create title with number of runs included
    title_text = f"Learning Curve Summary (n={num_runs} runs per experiment)"
    ax3.set_title(title_text, pad=25, fontsize=12)
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Combined plot saved to {output_path}")


def export_table_latex(summary_table, output_path, benchmark_name):
    """Export summary table to LaTeX format."""
    # Select relevant columns for LaTeX
    cols = ['experiment', 'train_steps', 'overall_success_mean', 'overall_success_std', 'num_runs']
    
    # Add metric columns
    metric_cols = [col for col in summary_table.columns if col.endswith('_mean') and not col.startswith('overall')]
    cols.extend(metric_cols)
    cols = [col for col in cols if col in summary_table.columns]
    
    table_latex = summary_table[cols].copy()
    
    latex = table_latex.to_latex(
        index=False,
        escape=True,
        longtable=False,
        caption=f"Learning curve summary for {benchmark_name.replace('_', ' ').title()}",
        label=f"tab:{benchmark_name}_summary",
        float_format="%.2f",
        na_rep="--",
        column_format="l" + "r" * (len(cols) - 1)
    )
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("% Auto-generated learning curve table\n\\begin{table}[t]\n\\centering\n" +
                           latex.split("\n", 1)[1].rsplit("\n", 1)[0] +
                           "\n\\end{table}\n")
    print(f"LaTeX table saved to {output_path}")


def export_table_csv(summary_table, output_path):
    """Export summary table to CSV format."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    summary_table.to_csv(output_path, index=False)
    print(f"CSV table saved to {output_path}")

def main():
    parser = argparse.ArgumentParser(description="Create learning curves from multiple experiment results.")
    parser.add_argument("benchmark_name", help="Benchmark name (e.g. benchmark_easy_default_cams)")
    parser.add_argument("experiment_dirs", nargs='+', help="Experiment directories with different training steps")
    parser.add_argument("--output", help="Output PNG file path (default: learning_curve_<benchmark_name>.png)")
    parser.add_argument("--output_dir", help="Output directory (default: parent of first experiment)")
    
    args = parser.parse_args()

    # Collect data from all experiments
    experiments_data = collect_experiments_data(args.experiment_dirs, args.benchmark_name)
    
    if not experiments_data:
        print("No valid experiment data found")
        return
    
    # Determine output path
    if args.output:
        output_path = args.output
    else:
        if args.output_dir:
            output_dir = args.output_dir
        else:
            output_dir = os.path.dirname(args.experiment_dirs[0])
        output_path = os.path.join(output_dir, f"learning_curve_{args.benchmark_name}.png")
    
    # Create summary table (now returns table and num_runs)
    summary_table, num_runs = create_summary_table(experiments_data)
    
    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir)
    else:
        output_dir = Path(args.experiment_dirs[0]).parent
    
    # Create individual plots
    overall_plot_path = output_dir / f"{args.benchmark_name}_overall_success.png"
    metrics_plot_path = output_dir / f"{args.benchmark_name}_metrics.png"
    combined_plot_path = output_dir / f"{args.benchmark_name}_combined.png"
    
    create_overall_success_plot(experiments_data, args.benchmark_name, overall_plot_path)
    create_metrics_plot(experiments_data, args.benchmark_name, metrics_plot_path)
    
    # Create combined plot with table
    create_combined_plot(experiments_data, args.benchmark_name, summary_table, combined_plot_path, num_runs)
    
    # Export table in LaTeX and CSV formats
    latex_path = output_dir / f"{args.benchmark_name}_table.tex"
    csv_path = output_dir / f"{args.benchmark_name}_table.csv"
    
    export_table_latex(summary_table, latex_path, args.benchmark_name)
    export_table_csv(summary_table, csv_path)
    
    # Also create the original combined plot for backward compatibility
    if args.output:
        create_learning_curve_plot(experiments_data, args.benchmark_name, args.output)
    
    # Print summary
    print(f"\nSummary:")
    print(f"Processed {len(experiments_data)} experiments")
    print(f"Training steps range: {min(exp['train_steps'] for exp in experiments_data):,} - {max(exp['train_steps'] for exp in experiments_data):,}")
    print(f"Success rate range: {min(exp['overall_success_mean'] for exp in experiments_data):.1f}% - {max(exp['overall_success_mean'] for exp in experiments_data):.1f}%")
    print(f"Number of benchmark runs per experiment: {min(exp['num_runs'] for exp in experiments_data)} - {max(exp['num_runs'] for exp in experiments_data)}")
    print(f"\nGenerated files in {output_dir}:")
    print(f"- {overall_plot_path.name}")
    print(f"- {metrics_plot_path.name}")
    print(f"- {combined_plot_path.name}")
    print(f"- {latex_path.name}")
    print(f"- {csv_path.name}")

if __name__ == "__main__":
    main()