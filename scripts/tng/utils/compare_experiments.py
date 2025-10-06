#!/usr/bin/env python3

import os
import glob
import json
import argparse
from collections import defaultdict
from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

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
        
        # Get benchmark summary
        summary = summarize_single_experiment(exp_dir, benchmark_name)
        
        if not summary:
            print(f"Warning: No benchmark data found for {exp_dir}")
            continue
            
        overall_success_mean = summary.get('overall_success_mean', 0) * 100
        overall_success_std = summary.get('overall_success_std', 0) * 100
        overall_success_rates = [rate * 100 for rate in summary.get('overall_success_rates', [])]
        
        # Extract metric rates
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
            'overall_success_mean': overall_success_mean,
            'overall_success_std': overall_success_std,
            'overall_success_rates': overall_success_rates,  # All individual run results
            'metric_rates_mean': metric_rates_mean,
            'metric_rates_all': metric_rates_all,  # All individual run results per metric
            'num_runs': len(overall_success_rates)
        })
        
        print(f"Processed {os.path.basename(exp_dir)}: {overall_success_mean:.1f}±{overall_success_std:.1f}% success ({len(overall_success_rates)} runs)")
    
    return experiments_data

def create_summary_table(experiments_data):
    """Create a summary table with success rate and metrics (no num runs row)."""
    
    # Create column headers (experiment names)
    columns = [exp['experiment_name'].replace('_RealFatMachine', '') for exp in experiments_data]
    
    # Collect all unique metrics
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    all_metrics = sorted(list(all_metrics))
    
    # Create table data
    table_data = {}
    
    # Row 1: Overall success rate (mean ± std format)
    success_rates = []
    for exp in experiments_data:
        mean = exp['overall_success_mean']
        std = exp['overall_success_std']
        success_rates.append(f"{mean:.1f}±{std:.1f}%")
    table_data['Success Rate'] = success_rates
    
    # Following rows: Individual metrics (mean ± std format)
    for metric in all_metrics:
        metric_values = []
        for exp in experiments_data:
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
    num_runs = experiments_data[0]['num_runs'] if experiments_data else 0
    
    return df, num_runs

def create_overall_success_plot(experiments_data, benchmark_name, output_path):
    """Create individual plot for overall success rate."""
    plt.rcParams.update({
        "pdf.fonttype": 42, "ps.fonttype": 42, "font.size": 10,
        "axes.labelsize": 11, "axes.titlesize": 11, "legend.fontsize": 9,
        "xtick.labelsize": 9, "ytick.labelsize": 9,
    })
    
    fig, ax = plt.subplots(figsize=(10, 4))
    
    experiment_names = [exp['experiment_name'].replace('_RealFatMachine', '') for exp in experiments_data]
    overall_success_mean = [exp['overall_success_mean'] for exp in experiments_data]
    overall_success_std = [exp['overall_success_std'] for exp in experiments_data]
    
    x_pos = np.arange(len(experiment_names))
    
    bars = ax.bar(x_pos, overall_success_mean, yerr=overall_success_std, 
                  capsize=5, color='steelblue', alpha=0.8, 
                  edgecolor='darkblue', linewidth=1.0)
    
    # Add value labels on bars
    for i, (bar, mean_val, std_val, exp) in enumerate(zip(bars, overall_success_mean, overall_success_std, experiments_data)):
        height = bar.get_height()
        text_y_position = height + std_val
        ax.annotate(f'{mean_val:.1f}±{std_val:.1f}%\n(n={exp["num_runs"]})',
                   xy=(bar.get_x() + bar.get_width() / 2, text_y_position),
                   xytext=(0, 8),
                   textcoords="offset points",
                   ha='center', va='bottom',
                   fontsize=8, fontweight='bold')
    
    ax.set_xlabel('Experiments')
    ax.set_ylabel('Overall Success Rate (%)')
    ax.set_title(f'Overall Success Rates: {benchmark_name.replace("_", " ").title()}')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(experiment_names, rotation=45, ha='right')
    ax.grid(True, alpha=0.3, axis='y')
    ax.set_ylim(0, max([m + s for m, s in zip(overall_success_mean, overall_success_std)]) * 1.3 if overall_success_mean else 100)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Overall success plot saved to {output_path}")

def create_metrics_plot(experiments_data, benchmark_name, output_path):
    """Create individual plot for metrics comparison."""
    plt.rcParams.update({
        "pdf.fonttype": 42, "ps.fonttype": 42, "font.size": 10,
        "axes.labelsize": 11, "axes.titlesize": 11, "legend.fontsize": 9,
        "xtick.labelsize": 9, "ytick.labelsize": 9,
    })
    
    # Collect all metrics
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    
    if not all_metrics:
        print("No metrics found, skipping metrics plot")
        return
    
    fig, ax = plt.subplots(figsize=(10, 4))
    
    experiment_names = [exp['experiment_name'].replace('_RealFatMachine', '') for exp in experiments_data]
    x_pos = np.arange(len(experiment_names))
    
    # Define colors using the same scheme as learning_curve.py
    colors = cm.get_cmap('tab10')(np.linspace(0, 1, 10))
    
    # Number of metrics and experiments
    n_metrics = len(all_metrics)
    bar_width = 0.8 / n_metrics
    
    max_metric_value = 0
    
    for i, metric in enumerate(sorted(all_metrics)):
        metric_means = []
        metric_stds = []
        
        for exp in experiments_data:
            mean_val = exp['metric_rates_mean'].get(metric, 0)
            individual_vals = exp['metric_rates_all'].get(metric, [])
            std_val = np.std(individual_vals) if len(individual_vals) > 1 else 0
            metric_means.append(mean_val)
            metric_stds.append(std_val)
        
        max_metric_value = max(max_metric_value, 
                             max([m + s for m, s in zip(metric_means, metric_stds)]) if metric_means else 0)
        
        # Position bars for this metric
        x_positions = x_pos + (i - n_metrics/2 + 0.5) * bar_width
        color = colors[i % len(colors)]
        
        ax.bar(x_positions, metric_means, bar_width, 
               yerr=metric_stds, capsize=3,
               color=color, alpha=0.8, 
               label=metric.replace('_', ' ').title(),
               edgecolor='black', linewidth=0.5)
    
    ax.set_xlabel('Experiments')
    ax.set_ylabel('Success Rate (%)')
    ax.set_title(f'Individual Metrics: {benchmark_name.replace("_", " ").title()}')
    ax.set_xticks(x_pos)
    ax.set_xticklabels(experiment_names, rotation=45, ha='right')
    ax.grid(True, alpha=0.3, axis='y')
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left', frameon=False)
    ax.set_ylim(0, max_metric_value * 1.2 if max_metric_value > 0 else 100)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Metrics plot saved to {output_path}")

def create_combined_plot(experiments_data, benchmark_name, summary_table, output_path, num_runs):
    """Create a combined plot with success rates, metrics, and summary table."""
    plt.rcParams.update({
        "pdf.fonttype": 42, "ps.fonttype": 42, "font.size": 9,
        "axes.labelsize": 10, "axes.titlesize": 11, "legend.fontsize": 8,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
    })
    
    # Check if we have metrics
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    
    # Create figure with 3 subplots: 2 plots on top, table on bottom
    fig = plt.figure(figsize=(16, 16))
    gs = fig.add_gridspec(3, 2, height_ratios=[2, 2, 1.5], hspace=0.8, wspace=0.3)
    
    ax1 = fig.add_subplot(gs[0, :])  # Overall success plot (spans both columns)
    ax2 = fig.add_subplot(gs[1, :]) if all_metrics else None # Metrics plot (spans both columns)
    ax3 = fig.add_subplot(gs[2, :])  # Table (spans both columns)
    
    experiment_names = [exp['experiment_name'].replace('_RealFatMachine', '') for exp in experiments_data]
    overall_success_mean = [exp['overall_success_mean'] for exp in experiments_data]
    overall_success_std = [exp['overall_success_std'] for exp in experiments_data]
    
    x_pos = np.arange(len(experiment_names))
    
    # Plot 1: Overall success rate
    bars1 = ax1.bar(x_pos, overall_success_mean, yerr=overall_success_std, 
                   capsize=5, color='steelblue', alpha=0.8, 
                   edgecolor='darkblue', linewidth=1.0)
    
    # Add value labels
    for i, (bar, mean_val, std_val, exp) in enumerate(zip(bars1, overall_success_mean, overall_success_std, experiments_data)):
        height = bar.get_height()
        text_y_position = height + std_val
        ax1.annotate(f'{mean_val:.1f}±{std_val:.1f}%\n(n={exp["num_runs"]})',
                    xy=(bar.get_x() + bar.get_width() / 2, text_y_position),
                    xytext=(0, 8),
                    textcoords="offset points",
                    ha='center', va='bottom',
                    fontsize=8, fontweight='bold')
    
    ax1.set_xlabel('Experiments')
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title(f'Overall Success Rates: {benchmark_name.replace("_", " ").title()}')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(experiment_names, rotation=45, ha='right')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, max([m + s for m, s in zip(overall_success_mean, overall_success_std)]) * 1.2 if overall_success_mean else 100)
    
    # Plot 2: Individual metrics
    if all_metrics and ax2 is not None:
        colors = cm.get_cmap('tab10')(np.linspace(0, 1, 10))
        n_metrics = len(all_metrics)
        bar_width = 0.8 / n_metrics
        max_metric_value = 0
        
        for i, metric in enumerate(sorted(all_metrics)):
            metric_means = []
            metric_stds = []
            
            for exp in experiments_data:
                mean_val = exp['metric_rates_mean'].get(metric, 0)
                individual_vals = exp['metric_rates_all'].get(metric, [])
                std_val = np.std(individual_vals) if len(individual_vals) > 1 else 0
                metric_means.append(mean_val)
                metric_stds.append(std_val)
            
            max_metric_value = max(max_metric_value, 
                                 max([m + s for m, s in zip(metric_means, metric_stds)]) if metric_means else 0)
            
            x_positions = x_pos + (i - n_metrics/2 + 0.5) * bar_width
            color = colors[i % len(colors)]
            
            ax2.bar(x_positions, metric_means, bar_width, 
                   yerr=metric_stds, capsize=3,
                   color=color, alpha=0.8, 
                   label=metric.replace('_', ' ').title(),
                   edgecolor='black', linewidth=0.5)
        
        ax2.set_xlabel('Experiments')
        ax2.set_ylabel('Success Rate (%)')
        ax2.set_title(f'Individual Metrics: {benchmark_name.replace("_", " ").title()}')
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels(experiment_names, rotation=45, ha='right')
        ax2.grid(True, alpha=0.3)
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left', frameon=False)
        ax2.set_ylim(0, max_metric_value * 1.2 if max_metric_value > 0 else 100)
    else:
        if ax2 is not None:
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
    title_text = f"Experiment Comparison Summary (n={num_runs} runs per experiment)"
    ax3.set_title(title_text, pad=25, fontsize=12)
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Combined plot saved to {output_path}")

def export_table_latex(summary_table, output_path, benchmark_name):
    """Export summary table to LaTeX format."""
    latex = summary_table.to_latex(
        escape=True,
        longtable=False,
        caption=f"Experiment comparison summary: {benchmark_name.replace('_', ' ').title()}",
        label=f"tab:{benchmark_name}_comparison",
        float_format="%.1f",
        na_rep="--"
    )
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("% Auto-generated by compare_experiments.py\n\\begin{table}[t]\n\\centering\n" +
                          latex.split("\n", 1)[1].rsplit("\n", 1)[0] +
                          "\n\\end{table}\n")
    print(f"LaTeX table saved to {output_path}")

def export_table_csv(summary_table, output_path):
    """Export summary table to CSV format."""
    summary_table.to_csv(output_path)
    print(f"CSV table saved to {output_path}")

def create_experiment_comparison_plot(experiments_data, benchmark_name, output_path):
    """Create a comparison plot showing success rates and metrics across experiments."""
    
    if not experiments_data:
        print("No experiment data to plot")
        return
    
    # Determine layout based on whether we have metrics
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    
    if all_metrics:
        # 1x3 grid: overall success, metrics, box plot
        fig, (ax1, ax2, ax_box) = plt.subplots(1, 3, figsize=(18, 6))
    else:
        # 1x2 grid: overall success left, box plot right
        fig, (ax1, ax_box) = plt.subplots(1, 2, figsize=(12, 6))
        ax2 = None
    
    # Extract data for plotting
    experiment_names = [exp['experiment_name'] for exp in experiments_data]
    overall_success_mean = [exp['overall_success_mean'] for exp in experiments_data]
    overall_success_std = [exp['overall_success_std'] for exp in experiments_data]
    
    # Create x positions for bars
    x_pos = np.arange(len(experiment_names))
    
    # Define consistent colors for metrics (used across all plots)
    metric_colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', 
                    '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']
    
    # Box plot: Distribution of means across experiments
    box_plot_data = []
    box_plot_labels = []
    box_plot_colors = []
    
    # Add overall success rate means
    box_plot_data.append(overall_success_mean)
    box_plot_labels.append('Overall')
    box_plot_colors.append('steelblue')
    
    # Add individual metric means with consistent colors
    if all_metrics:
        for i, metric in enumerate(sorted(all_metrics)):
            metric_means_across_experiments = []
            for exp in experiments_data:
                metric_means_across_experiments.append(exp['metric_rates_mean'].get(metric, 0))
            box_plot_data.append(metric_means_across_experiments)
            # Shorten metric names for better display
            short_name = metric.replace('_', ' ').title()
            if len(short_name) > 10:
                short_name = short_name[:10] + '...'
            box_plot_labels.append(short_name)
            box_plot_colors.append(metric_colors[i])
    
    # Create the box plot
    bp = ax_box.boxplot(box_plot_data, labels=box_plot_labels, patch_artist=True)
    
    # Apply consistent colors to box plots
    for i, patch in enumerate(bp['boxes']):
        patch.set_facecolor(box_plot_colors[i])
        patch.set_alpha(0.7)
    
    ax_box.set_ylabel('Success Rate (%)', fontsize=12, fontweight='bold')
    ax_box.set_title('Distribution of Means', fontsize=12, fontweight='bold')
    ax_box.tick_params(axis='x', rotation=45, labelsize=9)
    ax_box.grid(True, alpha=0.3, axis='y')
    
    # Plot 1: Overall success rate with error bars
    bars1 = ax1.bar(x_pos, overall_success_mean, yerr=overall_success_std, 
                    capsize=5, color='steelblue', alpha=0.8, 
                    edgecolor='darkblue', linewidth=1.5)
    
    # Add value labels on bars
    for i, (bar, mean_val, std_val, exp) in enumerate(zip(bars1, overall_success_mean, overall_success_std, experiments_data)):
        height = bar.get_height()
        # Position text above error bars by adding the standard deviation to the height
        text_y_position = height + std_val
        ax1.annotate(f'{mean_val:.1f}±{std_val:.1f}%\n(n={exp["num_runs"]})',
                    xy=(bar.get_x() + bar.get_width() / 2, text_y_position),
                    xytext=(0, 8),  # Additional offset above the error bars
                    textcoords="offset points",
                    ha='center', va='bottom',
                    fontsize=8, fontweight='bold')
    
    ax1.set_xlabel('Experiments', fontsize=12, fontweight='bold')
    ax1.set_ylabel('Overall Success Rate (%)', fontsize=12, fontweight='bold')
    ax1.set_title('Overall Success Rates', fontsize=14, fontweight='bold')
    ax1.set_xticks(x_pos)
    ax1.set_xticklabels(experiment_names, rotation=45, ha='right')
    ax1.grid(True, alpha=0.3, axis='y')
    ax1.set_ylim(0, max(overall_success_mean) * 1.3 if overall_success_mean else 100)
    
    # Plot 2: Individual metrics comparison
    if all_metrics and ax2 is not None:
        # Number of metrics and experiments
        n_metrics = len(all_metrics)
        n_experiments = len(experiments_data)
        
        # Create grouped bar chart
        bar_width = 0.8 / n_metrics
        
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
            
            max_metric_value = max(max_metric_value, 
                                 max([m + s for m, s in zip(metric_means, metric_stds)]) if metric_means else 0)
            
            # Position bars for this metric
            x_positions = x_pos + (i - n_metrics/2 + 0.5) * bar_width
            
            bars2 = ax2.bar(x_positions, metric_means, bar_width, 
                           yerr=metric_stds, capsize=3,
                           color=metric_colors[i], alpha=0.8, 
                           label=metric.replace('_', ' ').title(),
                           edgecolor='black', linewidth=0.5)
        
        ax2.set_xlabel('Experiments', fontsize=12, fontweight='bold')
        ax2.set_ylabel('Success Rate (%)', fontsize=12, fontweight='bold')
        ax2.set_title('Individual Metrics Comparison (with uncertainty)', 
                      fontsize=12, fontweight='bold')
        ax2.set_xticks(x_pos)
        ax2.set_xticklabels(experiment_names, rotation=45, ha='right')
        ax2.grid(True, alpha=0.3, axis='y')
        ax2.legend(bbox_to_anchor=(0.5, -0.35), loc='upper center', ncol=3)
        ax2.set_ylim(0, max_metric_value * 1.2 if max_metric_value > 0 else 100)
    
    # Add figure title with proper spacing
    fig.suptitle(f'Experiment Comparison: {benchmark_name.replace("_", " ").title()}', 
                 fontsize=16, fontweight='bold')
    
    # Adjust layout with closer spacing between plots
    plt.tight_layout(pad=1.0)
    plt.subplots_adjust(top=0.88, wspace=0.2)  # Leave space for title, reduce horizontal spacing
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Experiment comparison plot saved to {output_path}")
    return output_path

def print_detailed_comparison(experiments_data, benchmark_name):
    """Print detailed comparison statistics."""
    print(f"\n{'='*80}")
    print(f"DETAILED COMPARISON REPORT: {benchmark_name.replace('_', ' ').title()}")
    print(f"{'='*80}")
    
    # Overall success rates
    print(f"\nOVERALL SUCCESS RATES:")
    print(f"{'Experiment':<30} {'Mean ± Std (%)':<15} {'Runs':<6} {'Min (%)':<8} {'Max (%)':<8}")
    print(f"{'-'*80}")
    
    for exp in experiments_data:
        rates = exp['overall_success_rates']
        min_rate = min(rates) if rates else 0
        max_rate = max(rates) if rates else 0
        print(f"{exp['experiment_name']:<30} {exp['overall_success_mean']:>6.1f}±{exp['overall_success_std']:<6.1f} {exp['num_runs']:<6} {min_rate:<8.1f} {max_rate:<8.1f}")
    
    # Individual metrics
    all_metrics = set()
    for exp in experiments_data:
        all_metrics.update(exp['metric_rates_mean'].keys())
    
    if all_metrics:
        print(f"\nINDIVIDUAL METRICS:")
        for metric in sorted(all_metrics):
            print(f"\n{metric.replace('_', ' ').title()}:")
            print(f"{'Experiment':<30} {'Mean ± Std (%)':<15} {'Min (%)':<8} {'Max (%)':<8}")
            print(f"{'-'*70}")
            
            for exp in experiments_data:
                if metric in exp['metric_rates_mean']:
                    rates = exp['metric_rates_all'][metric]
                    mean_val = exp['metric_rates_mean'][metric]
                    std_val = np.std(rates) if len(rates) > 1 else 0
                    min_rate = min(rates) if rates else 0
                    max_rate = max(rates) if rates else 0
                    print(f"{exp['experiment_name']:<30} {mean_val:>6.1f}±{std_val:<6.1f} {min_rate:<8.1f} {max_rate:<8.1f}")
                else:
                    print(f"{exp['experiment_name']:<30} {'N/A':<15} {'N/A':<8} {'N/A':<8}")

def main():
    parser = argparse.ArgumentParser(description="Compare multiple experiments on a single benchmark.")
    parser.add_argument("benchmark_name", help="Benchmark name (e.g. benchmark_easy_default_cams)")
    parser.add_argument("experiment_dirs", nargs='+', help="Experiment directories to compare")
    parser.add_argument("--output_dir", help="Output directory (default: parent of first experiment dir)")
    parser.add_argument("--detailed", action="store_true", help="Print detailed comparison statistics")
    
    args = parser.parse_args()

    # Collect data from all experiments
    experiments_data = collect_experiments_data(args.experiment_dirs, args.benchmark_name)
    
    if not experiments_data:
        print("No valid experiment data found")
        return
    
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
    legacy_plot_path = output_dir / f"comparison_{args.benchmark_name}.png"  # Keep legacy name too
    
    create_overall_success_plot(experiments_data, args.benchmark_name, overall_plot_path)
    create_metrics_plot(experiments_data, args.benchmark_name, metrics_plot_path)
    
    # Create combined plot with table
    create_combined_plot(experiments_data, args.benchmark_name, summary_table, combined_plot_path, num_runs)
    
    # Create legacy format plot for backward compatibility
    create_experiment_comparison_plot(experiments_data, args.benchmark_name, legacy_plot_path)
    
    # Export table in LaTeX and CSV formats
    latex_path = output_dir / f"{args.benchmark_name}_table.tex"
    csv_path = output_dir / f"{args.benchmark_name}_table.csv"
    
    export_table_latex(summary_table, latex_path, args.benchmark_name)
    export_table_csv(summary_table, csv_path)
    
    # Print detailed comparison if requested
    if args.detailed:
        print_detailed_comparison(experiments_data, args.benchmark_name)
    
    # Print summary
    print(f"\nSUMMARY:")
    print(f"Compared {len(experiments_data)} experiments")
    success_rates = [exp['overall_success_mean'] for exp in experiments_data]
    if success_rates:
        print(f"Success rate range: {min(success_rates):.1f}% - {max(success_rates):.1f}%")
        best_exp = max(experiments_data, key=lambda x: x['overall_success_mean'])
        print(f"Best performing experiment: {best_exp['experiment_name']} ({best_exp['overall_success_mean']:.1f}%)")
    
    runs_per_exp = [exp['num_runs'] for exp in experiments_data]
    print(f"Benchmark runs per experiment: {min(runs_per_exp)} - {max(runs_per_exp)}")
    
    print(f"\nOUTPUTS CREATED:")
    print(f"- Individual plots: {overall_plot_path}, {metrics_plot_path}")
    print(f"- Combined plot: {combined_plot_path}")
    print(f"- Legacy plot: {legacy_plot_path}")
    print(f"- Table exports: {latex_path}, {csv_path}")

if __name__ == "__main__":
    main()