#!/usr/bin/env python3

import os
import glob
import json
import argparse
import re
from collections import defaultdict
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

def create_learning_curve_plot(experiments_data, benchmark_name, output_path):
    """Create a learning curve plot showing success rates vs training steps with uncertainty."""
    
    if not experiments_data:
        print("No experiment data to plot")
        return
    
    # Create figure with 2 subplots: overall success and individual metrics
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Extract data for plotting
    train_steps = [exp['train_steps'] for exp in experiments_data]
    overall_success_mean = [exp['overall_success_mean'] for exp in experiments_data]
    overall_success_std = [exp['overall_success_std'] for exp in experiments_data]
    
    # Plot 1: Overall success rate with error bars
    ax1.errorbar(train_steps, overall_success_mean, yerr=overall_success_std, 
                fmt='o-', linewidth=3, markersize=8, capsize=5, capthick=2,
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
    
    ax1.set_xlabel('Training Steps', fontsize=12)
    ax1.set_ylabel('Success Rate (%)', fontsize=12)
    ax1.set_title(f'Learning Curve: {benchmark_name.replace("_", " ").title()}', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
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
        colors = plt.cm.Set1(np.linspace(0, 1, len(all_metrics)))
        
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
                        fmt='o-', linewidth=2, markersize=6, capsize=3,
                        color=colors[i], label=metric.replace('_', ' ').title(), alpha=0.8)
        
        ax2.set_ylabel('Success Rate (%)', fontsize=12)
        ax2.set_title('Individual Metrics Learning Curves (with uncertainty)', fontsize=12, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax2.set_ylim(0, max_metric_value * 1.1 if max_metric_value > 0 else 100)
        
        # Format x-axis for metrics plot
        if max(train_steps) >= 1000:
            ax2.set_xlabel('Training Steps (×1000)', fontsize=12)
            ax2.set_xticks(train_steps)
            ax2.set_xticklabels([f'{int(s/1000)}k' for s in train_steps])
        else:
            ax2.set_xlabel('Training Steps', fontsize=12)
    else:
        # Hide second subplot if no metrics
        ax2.set_visible(False)
        fig.set_size_inches(12, 6)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Learning curve saved to {output_path}")
    return output_path

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
    
    # Create the plot
    create_learning_curve_plot(experiments_data, args.benchmark_name, output_path)
    
    # Print summary
    print(f"\nSummary:")
    print(f"Processed {len(experiments_data)} experiments")
    print(f"Training steps range: {min(exp['train_steps'] for exp in experiments_data):,} - {max(exp['train_steps'] for exp in experiments_data):,}")
    print(f"Success rate range: {min(exp['overall_success_mean'] for exp in experiments_data):.1f}% - {max(exp['overall_success_mean'] for exp in experiments_data):.1f}%")
    print(f"Number of benchmark runs per experiment: {min(exp['num_runs'] for exp in experiments_data)} - {max(exp['num_runs'] for exp in experiments_data)}")

if __name__ == "__main__":
    main()