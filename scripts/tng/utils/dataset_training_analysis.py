#!/usr/bin/env python3

import os
import glob
import json
import argparse
import re
from collections import defaultdict
import matplotlib.pyplot as plt
import numpy as np

def extract_experiment_params(experiment_dir):
    """Extract both training steps and dataset size from experiment directory name."""
    dir_name = os.path.basename(experiment_dir)
    
    # Extract training steps
    train_steps = None
    train_patterns = [
        r'_(\d+)k',  # train_steps_100k
    ]
    
    for pattern in train_patterns:
        match = re.search(pattern, dir_name, re.IGNORECASE)
        if match:
            steps = int(match.group(1))
            if 'k' in pattern:
                steps *= 1000
            train_steps = steps
            break
    
    # Extract dataset size
    dataset_size = None
    dataset_patterns = [
        r'dataset_size_(\d+)',     # dataset_size_100
        r'data_(\d+)',             # data_100
        r'size_(\d+)',             # size_100
        r'ds_(\d+)',               # ds_100
        r'(\d+)_samples',          # 100_samples
    ]
    
    for pattern in dataset_patterns:
        match = re.search(pattern, dir_name, re.IGNORECASE)
        if match:
            dataset_size = int(match.group(1))
            break
    
    # If no specific patterns found, try to extract two numbers and make educated guess
    if train_steps is None or dataset_size is None:
        numbers = re.findall(r'\d+', dir_name)
        if len(numbers) >= 2:
            # Heuristic: larger number is likely training steps, smaller is dataset size
            nums = [int(n) for n in numbers]
            nums.sort()
            if dataset_size is None:
                dataset_size = nums[0] if nums[0] < 10000 else nums[-2] if len(nums) > 1 else nums[0]
            if train_steps is None:
                train_steps = nums[-1] if nums[-1] > 1000 else nums[-1] * 1000
    
    return train_steps, dataset_size

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
        "overall_success_rates": overall_rates,
        "metric_rates": {}
    }
    
    if overall_rates:
        summary["overall_success_mean"] = sum(overall_rates) / len(overall_rates)
        summary["overall_success_std"] = np.std(overall_rates) if len(overall_rates) > 1 else 0
    
    for metric, rates in metric_totals.items():
        summary["metric_rates"][metric] = rates
        summary[f"metric_{metric}_mean"] = sum(rates) / len(rates)
        summary[f"metric_{metric}_std"] = np.std(rates) if len(rates) > 1 else 0

    return summary

def collect_experiments_grid_data(experiment_dirs, benchmark_name):
    """Collect data organized by training steps and dataset size."""
    experiments_data = []
    
    for exp_dir in experiment_dirs:
        if not os.path.exists(exp_dir):
            print(f"Warning: Directory {exp_dir} does not exist")
            continue
            
        # Extract both parameters
        train_steps, dataset_size = extract_experiment_params(exp_dir)
        if train_steps is None or dataset_size is None:
            print(f"Warning: Could not extract parameters from {exp_dir}")
            print(f"  Extracted: train_steps={train_steps}, dataset_size={dataset_size}")
            continue
        
        # Get benchmark summary
        summary = summarize_single_experiment(exp_dir, benchmark_name)
        
        if not summary:
            print(f"Warning: No benchmark data found for {exp_dir}")
            continue
            
        overall_success_mean = summary.get('overall_success_mean', 0) * 100
        overall_success_std = summary.get('overall_success_std', 0) * 100
        overall_success_rates = [rate * 100 for rate in summary.get('overall_success_rates', [])]
        
        experiments_data.append({
            'experiment_dir': exp_dir,
            'experiment_name': os.path.basename(exp_dir),
            'train_steps': train_steps,
            'dataset_size': dataset_size,
            'overall_success_mean': overall_success_mean,
            'overall_success_std': overall_success_std,
            'overall_success_rates': overall_success_rates,
            'num_runs': len(overall_success_rates)
        })
        
        print(f"Processed {os.path.basename(exp_dir)}: {train_steps} steps, {dataset_size} samples, {overall_success_mean:.1f}±{overall_success_std:.1f}% success ({len(overall_success_rates)} runs)")
    
    return experiments_data

def create_grid_heatmap(experiments_data, benchmark_name, output_path):
    """Create color-graded grid plot (heatmap) with annotations."""
    
    if not experiments_data:
        print("No experiment data to plot")
        return
    
    # Extract unique values for grid
    train_steps_unique = sorted(list(set(exp['train_steps'] for exp in experiments_data)))
    dataset_sizes_unique = sorted(list(set(exp['dataset_size'] for exp in experiments_data)))
    
    print(f"Training steps: {train_steps_unique}")
    print(f"Dataset sizes: {dataset_sizes_unique}")
    
    # Create matrices for the heatmap
    success_matrix = np.full((len(dataset_sizes_unique), len(train_steps_unique)), np.nan)
    std_matrix = np.full((len(dataset_sizes_unique), len(train_steps_unique)), np.nan)
    runs_matrix = np.full((len(dataset_sizes_unique), len(train_steps_unique)), 0)
    
    # Fill matrices with data
    for exp in experiments_data:
        steps_idx = train_steps_unique.index(exp['train_steps'])
        size_idx = dataset_sizes_unique.index(exp['dataset_size'])
        success_matrix[size_idx, steps_idx] = exp['overall_success_mean']
        std_matrix[size_idx, steps_idx] = exp['overall_success_std']
        runs_matrix[size_idx, steps_idx] = exp['num_runs']
    
    # Create figure
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Create heatmap
    im = ax.imshow(success_matrix, cmap='RdYlGn', aspect='auto', origin='lower',
                   vmin=0, vmax=100, interpolation='nearest')
    
    # Set ticks and labels
    ax.set_xticks(range(len(train_steps_unique)))
    ax.set_yticks(range(len(dataset_sizes_unique)))
    
    # Format x-axis labels for training steps
    if max(train_steps_unique) >= 1000:
        ax.set_xticklabels([f'{int(s/1000)}k' for s in train_steps_unique])
        ax.set_xlabel('Training Steps (×1000)', fontsize=14)
    else:
        ax.set_xticklabels(train_steps_unique)
        ax.set_xlabel('Training Steps', fontsize=14)
    
    ax.set_yticklabels(dataset_sizes_unique)
    ax.set_ylabel('Dataset Size', fontsize=14)
    ax.set_title(f'Performance Grid: {benchmark_name.replace("_", " ").title()}', 
                fontsize=16, fontweight='bold', pad=20)
    
    # Add text annotations with detailed stats
    for i in range(len(dataset_sizes_unique)):
        for j in range(len(train_steps_unique)):
            if not np.isnan(success_matrix[i, j]):
                success = success_matrix[i, j]
                std = std_matrix[i, j]
                n_runs = runs_matrix[i, j]
                
                # Choose text color based on background
                text_color = 'white' if success < 50 else 'black'
                
                # Create annotation text
                annotation_text = f'{success:.1f}±{std:.1f}%\n(n={n_runs})'
                
                ax.text(j, i, annotation_text, 
                       ha='center', va='center', fontweight='bold',
                       color=text_color, fontsize=10)
    
    # Add colorbar
    cbar = plt.colorbar(im, ax=ax, shrink=0.8, pad=0.02)
    cbar.set_label('Success Rate (%)', fontsize=12)
    cbar.ax.tick_params(labelsize=10)
    
    # Add grid lines for better separation
    ax.set_xticks(np.arange(-0.5, len(train_steps_unique), 1), minor=True)
    ax.set_yticks(np.arange(-0.5, len(dataset_sizes_unique), 1), minor=True)
    ax.grid(which='minor', color='white', linestyle='-', linewidth=2)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    print(f"Dataset-Training grid analysis saved to {output_path}")
    return output_path

def main():
    parser = argparse.ArgumentParser(description="Analyze performance across training steps and dataset sizes.")
    parser.add_argument("benchmark_name", help="Benchmark name (e.g. benchmark_easy_default_cams)")
    parser.add_argument("experiment_dirs", nargs='+', help="Experiment directories with different training steps and dataset sizes")
    parser.add_argument("--output", help="Output PNG file path (default: dataset_training_analysis_<benchmark_name>.png)")
    parser.add_argument("--output_dir", help="Output directory (default: parent of first experiment)")
    
    args = parser.parse_args()

    # Collect data from all experiments
    experiments_data = collect_experiments_grid_data(args.experiment_dirs, args.benchmark_name)
    
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
        output_path = os.path.join(output_dir, f"dataset_training_analysis_{args.benchmark_name}.png")
    
    # Create the plot
    create_grid_heatmap(experiments_data, args.benchmark_name, output_path)
    
    # Print summary
    train_steps_unique = sorted(list(set(exp['train_steps'] for exp in experiments_data)))
    dataset_sizes_unique = sorted(list(set(exp['dataset_size'] for exp in experiments_data)))
    
    print(f"\nSummary:")
    print(f"Processed {len(experiments_data)} experiments")
    print(f"Training steps: {train_steps_unique}")
    print(f"Dataset sizes: {dataset_sizes_unique}")
    print(f"Success rate range: {min(exp['overall_success_mean'] for exp in experiments_data):.1f}% - {max(exp['overall_success_mean'] for exp in experiments_data):.1f}%")
    print(f"Grid completeness: {len(experiments_data)}/{len(train_steps_unique) * len(dataset_sizes_unique)} combinations")

if __name__ == "__main__":
    main()