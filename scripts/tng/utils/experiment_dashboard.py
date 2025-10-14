#!/usr/bin/env python3
"""
Interactive Experiment Dashboard

A comprehensive Streamlit-based dashboard for browsing and visualizing 
robotics experiment results. Integrates all existing visualization tools
into a single interactive interface.

Usage:
    streamlit run experiment_dashboard.py

Features:
- Browse experiment directories interactively
- Generate learning curves
- Compare multiple experiments
- Visualize dataset distributions
- View TensorBoard logs
- Display benchmark summaries
- Dynamic plot generation and caching
"""

import streamlit as st
import os
import sys
import glob
import json
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib
import pandas as pd
import numpy as np
from io import BytesIO
import base64
import yaml
from collections import defaultdict
import re
from mpl_toolkits.mplot3d import Axes3D

# Set matplotlib backend for streamlit
matplotlib.use('Agg')

# Import existing utility modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from learning_curve import (
    collect_experiments_data as collect_learning_curve_data,
    create_overall_success_plot,
    create_metrics_plot as create_learning_metrics_plot,
    create_combined_plot as create_learning_combined_plot,
    create_summary_table as create_learning_summary_table
)
from compare_experiments import (
    collect_experiments_data as collect_comparison_data,
    create_overall_success_plot as create_comparison_success_plot,
    create_metrics_plot as create_comparison_metrics_plot,
    create_combined_plot as create_comparison_combined_plot,
    create_summary_table as create_comparison_summary_table
)
from visualize_dataset_distr import (
    load_positions,
    extract_coordinates,
    infer_dim
)
from summarize_benchmarks import (
    collect_benchmark_results,
    calculate_success_rates_per_position,
    create_benchmark_visualization
)
from plot_tensorboard_logs import (
    load_scalars,
    summarize_runs
)

# Configure Streamlit page
st.set_page_config(
    page_title="Isaac Lab Experiment Dashboard",
    page_icon="🤖",
    layout="wide",
    initial_sidebar_state="expanded"
)

# Custom CSS for better styling
st.markdown("""
<style>
    .main-header {
        font-size: 2.5rem;
        font-weight: bold;
        color: #1f77b4;
        text-align: center;
        margin-bottom: 2rem;
    }
    .section-header {
        font-size: 1.5rem;
        font-weight: bold;
        color: #2ca02c;
        margin-top: 2rem;
        margin-bottom: 1rem;
        border-bottom: 2px solid #2ca02c;
        padding-bottom: 0.5rem;
    }
    .metric-box {
        background-color: #f0f2f6;
        padding: 1rem;
        border-radius: 0.5rem;
        margin: 0.5rem 0;
    }
    .experiment-card {
        border: 1px solid #ddd;
        border-radius: 0.5rem;
        padding: 1rem;
        margin: 0.5rem 0;
        background-color: #fafafa;
    }
</style>
""", unsafe_allow_html=True)

@st.cache_data
def scan_experiment_directories(base_dir):
    """Scan for experiment directories and their structure."""
    experiment_dirs = []
    
    if not os.path.exists(base_dir):
        return experiment_dirs
    
    for item in os.listdir(base_dir):
        item_path = os.path.join(base_dir, item)
        if os.path.isdir(item_path):
            # Check if this looks like an experiment directory
            has_models = any(d.startswith('models_') for d in os.listdir(item_path) if os.path.isdir(os.path.join(item_path, d)))
            has_eval = any(d.startswith('eval_') for d in os.listdir(item_path) if os.path.isdir(os.path.join(item_path, d)))
            has_data_viz = os.path.exists(os.path.join(item_path, 'data_visualization'))
            
            if has_models or has_eval or has_data_viz:
                experiment_dirs.append(item_path)
    
    return sorted(experiment_dirs)

@st.cache_data  
def get_experiment_info(exp_dir):
    """Get metadata about an experiment directory."""
    info = {
        'name': os.path.basename(exp_dir),
        'path': exp_dir,
        'models': [],
        'benchmarks': [],
        'datasets': [],
        'has_tensorboard': False,
        'has_data_viz': False
    }
    
    if not os.path.exists(exp_dir):
        return info
    
    # Scan for models
    for item in os.listdir(exp_dir):
        item_path = os.path.join(exp_dir, item)
        if os.path.isdir(item_path):
            if item.startswith('models_'):
                info['models'].append(item)
                # Check for tensorboard logs
                tb_files = glob.glob(os.path.join(item_path, '**/events.out.tfevents.*'), recursive=True)
                if tb_files:
                    info['has_tensorboard'] = True
            elif item.startswith('eval_'):
                # Handle naming convention: eval_<eval_number>_<benchmark_name>_from_checkpoint-<checkpoint_number>
                # Extract benchmark name by removing eval prefix and eval number, then split on _from_
                parts = item.replace('eval_', '').split('_from_')[0]
                # Remove the eval number (first part after eval_)
                benchmark_parts = parts.split('_')
                if len(benchmark_parts) > 1:
                    # Skip the first part (eval number) and rejoin the rest
                    benchmark_name = '_'.join(benchmark_parts[1:])
                else:
                    benchmark_name = parts
                
                if benchmark_name not in info['benchmarks']:
                    info['benchmarks'].append(benchmark_name)
            elif item == 'data_visualization':
                info['has_data_viz'] = True
                yaml_files = glob.glob(os.path.join(item_path, '*.yaml'))
                for yaml_file in yaml_files:
                    dataset_name = os.path.basename(yaml_file).replace('.yaml', '')
                    info['datasets'].append(dataset_name)
    
    return info

@st.cache_data
def get_available_benchmarks():
    """Get list of available benchmark files."""
    benchmark_dir = "/home/innovation-hacking/luebbet/dev/IsaacLab/tng_benchmarks"
    benchmarks = []
    
    if os.path.exists(benchmark_dir):
        for file in os.listdir(benchmark_dir):
            if file.endswith('.yaml'):
                benchmarks.append(file.replace('.yaml', ''))
    
    return sorted(benchmarks)

def create_matplotlib_figure_download(fig, filename):
    """Create a download button for matplotlib figure."""
    buffer = BytesIO()
    fig.savefig(buffer, format='png', dpi=300, bbox_inches='tight')
    buffer.seek(0)
    
    st.download_button(
        label=f"📥 Download {filename}",
        data=buffer.getvalue(),
        file_name=f"{filename}.png",
        mime="image/png"
    )

def extract_camera_poses(yaml_path):
    """Extract camera poses from YAML file."""
    with open(yaml_path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    
    cases = doc.get("test_cases", [])
    camera_data = {
        'main': {'positions': [], 'rotations': []},
        'secondary': {'positions': [], 'rotations': []}, 
        'wrist': {'positions': [], 'rotations': []}
    }
    
    for case in cases:
        # Extract camera pose data for each camera type
        for camera_type in ['main', 'secondary', 'wrist']:
            pose_key = f"camera_pose_{camera_type}"
            if pose_key in case:
                pose = case[pose_key]
                if 'pos' in pose:
                    camera_data[camera_type]['positions'].append(tuple(pose['pos']))
                if 'rpy' in pose:
                    camera_data[camera_type]['rotations'].append(tuple(pose['rpy']))
    
    return camera_data

def extract_colors(yaml_path):
    """Extract object and target colors from YAML file and map to color palette."""
    # Import the Color enum from colors.py
    sys.path.append('/home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils')
    from colors import Color
    
    with open(yaml_path, "r", encoding="utf-8") as f:
        doc = yaml.safe_load(f)
    
    cases = doc.get("test_cases", [])
    
    def rgb_to_color_name(rgb_tuple):
        """Map RGB tuple to color name from palette."""
        rgb = tuple(rgb_tuple)
        
        # Find closest color in palette
        min_distance = float('inf')
        closest_color = None
        
        for color in Color:
            color_rgb = color.rgb
            # Calculate Euclidean distance
            distance = sum((a - b) ** 2 for a, b in zip(rgb, color_rgb)) ** 0.5
            if distance < min_distance:
                min_distance = distance
                closest_color = color
        
        return closest_color.name if closest_color else "UNKNOWN"
    
    object_colors = []
    target_colors = []
    
    for case in cases:
        # Extract object color
        if 'object_rgb' in case:
            color_name = rgb_to_color_name(case['object_rgb'])
            object_colors.append(color_name)
        
        # Extract target color  
        if 'target_rgb' in case:
            color_name = rgb_to_color_name(case['target_rgb'])
            target_colors.append(color_name)
    
    return object_colors, target_colors

def create_camera_pose_visualization(camera_data, selected_dataset):
    """Create camera pose distribution visualization."""
    fig = plt.figure(figsize=(18, 12))
    
    # Position analysis (3D scatter plots)
    for i, (camera_type, data) in enumerate(camera_data.items()):
        if not data['positions']:
            continue
            
        positions = data['positions']
        x_vals = np.array([pos[0] for pos in positions])
        y_vals = np.array([pos[1] for pos in positions])
        z_vals = np.array([pos[2] for pos in positions])
        
        # 3D scatter plot for positions
        ax = fig.add_subplot(2, 3, i+1, projection='3d')
        scatter = ax.scatter(x_vals, y_vals, z_vals, alpha=0.7, s=50)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title(f'{camera_type.title()} Camera Positions')
        
        # Rotation histograms
        if data['rotations']:
            rotations = data['rotations']
            roll_vals = np.array([rot[0] for rot in rotations])
            pitch_vals = np.array([rot[1] for rot in rotations])
            yaw_vals = np.array([rot[2] for rot in rotations])
            
            ax_hist = fig.add_subplot(2, 3, i+4)
            ax_hist.hist([roll_vals, pitch_vals, yaw_vals], 
                        bins=20, alpha=0.7, 
                        label=['Roll', 'Pitch', 'Yaw'],
                        color=['red', 'green', 'blue'])
            ax_hist.set_xlabel('Angle (degrees)')
            ax_hist.set_ylabel('Frequency')
            ax_hist.set_title(f'{camera_type.title()} Camera Rotations')
            ax_hist.legend()
            ax_hist.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def create_color_distribution_visualization(object_colors, target_colors, selected_dataset):
    """Create color distribution visualization."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Object colors
    if object_colors:
        from collections import Counter
        obj_counts = Counter(object_colors)
        colors_obj = list(obj_counts.keys())
        counts_obj = list(obj_counts.values())
        
        bars1 = ax1.bar(colors_obj, counts_obj, alpha=0.8)
        ax1.set_xlabel('Object Colors')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Object Color Distribution')
        ax1.tick_params(axis='x', rotation=45)
        ax1.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, count in zip(bars1, counts_obj):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    str(count), ha='center', va='bottom')
    
    # Target colors
    if target_colors:
        from collections import Counter
        tgt_counts = Counter(target_colors)
        colors_tgt = list(tgt_counts.keys())
        counts_tgt = list(tgt_counts.values())
        
        bars2 = ax2.bar(colors_tgt, counts_tgt, alpha=0.8, color='orange')
        ax2.set_xlabel('Target Colors')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Target Color Distribution')
        ax2.tick_params(axis='x', rotation=45)
        ax2.grid(True, alpha=0.3)
        
        # Add value labels on bars
        for bar, count in zip(bars2, counts_tgt):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    str(count), ha='center', va='bottom')
    
    plt.tight_layout()
    return fig

# Benchmark-specific helper functions
def extract_benchmark_colors(cases):
    """Extract object and target colors from benchmark cases and map to color palette."""
    # Import the Color enum from colors.py
    sys.path.append('/home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils')
    from colors import Color
    
    def rgb_to_color_name(rgb_tuple):
        """Map RGB tuple to color name from palette."""
        rgb = tuple(rgb_tuple)
        
        # Find closest color in palette
        min_distance = float('inf')
        closest_color = None
        
        for color in Color:
            color_rgb = color.rgb
            # Calculate Euclidean distance
            distance = sum((a - b) ** 2 for a, b in zip(rgb, color_rgb)) ** 0.5
            if distance < min_distance:
                min_distance = distance
                closest_color = color
        
        return closest_color.name if closest_color else "UNKNOWN"
    
    object_colors = []
    target_colors = []
    
    for case in cases:
        # Extract object color
        if 'object_rgb' in case:
            color_name = rgb_to_color_name(case['object_rgb'])
            object_colors.append(color_name)
        
        # Extract target color  
        if 'target_rgb' in case:
            color_name = rgb_to_color_name(case['target_rgb'])
            target_colors.append(color_name)
    
    return object_colors, target_colors

def extract_benchmark_camera_poses(cases):
    """Extract camera poses from benchmark cases."""
    camera_data = {
        'main': {'positions': [], 'rotations': []},
        'secondary': {'positions': [], 'rotations': []}, 
        'wrist': {'positions': [], 'rotations': []}
    }
    
    for case in cases:
        # Extract camera pose data for each camera type
        for camera_type in ['main', 'secondary', 'wrist']:
            pose_key = f"camera_pose_{camera_type}"
            if pose_key in case:
                pose = case[pose_key]
                if 'pos' in pose:
                    camera_data[camera_type]['positions'].append(tuple(pose['pos']))
                if 'rpy' in pose:
                    camera_data[camera_type]['rotations'].append(tuple(pose['rpy']))
    
    return camera_data

def create_benchmark_position_visualization(obj_positions, tgt_positions, cases, benchmark_name):
    """Create position distribution visualization for benchmark results."""
    fig = plt.figure(figsize=(16, 10))
    
    # Extract success information
    successes = [case.get('success', False) for case in cases]
    
    # 2D scatter plot of positions
    ax1 = fig.add_subplot(2, 2, 1)
    
    # Separate successful and failed cases
    success_obj = [pos for pos, success in zip(obj_positions, successes) if success]
    success_tgt = [pos for pos, success in zip(tgt_positions, successes) if success]
    fail_obj = [pos for pos, success in zip(obj_positions, successes) if not success]
    fail_tgt = [pos for pos, success in zip(tgt_positions, successes) if not success]
    
    # Plot object positions
    if success_obj:
        ax1.scatter([pos[0] for pos in success_obj], [pos[1] for pos in success_obj], 
                   c='green', marker='o', alpha=0.7, s=50, label='Successful Object')
    if fail_obj:
        ax1.scatter([pos[0] for pos in fail_obj], [pos[1] for pos in fail_obj], 
                   c='red', marker='o', alpha=0.7, s=50, label='Failed Object')
    
    # Plot target positions  
    if success_tgt:
        ax1.scatter([pos[0] for pos in success_tgt], [pos[1] for pos in success_tgt], 
                   c='darkgreen', marker='^', alpha=0.7, s=50, label='Successful Target')
    if fail_tgt:
        ax1.scatter([pos[0] for pos in fail_tgt], [pos[1] for pos in fail_tgt], 
                   c='darkred', marker='^', alpha=0.7, s=50, label='Failed Target')
    
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title(f'Position Distribution - {benchmark_name}')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')
    
    # X-position distribution
    ax2 = fig.add_subplot(2, 2, 2)
    obj_x = [pos[0] for pos in obj_positions]
    tgt_x = [pos[0] for pos in tgt_positions]
    
    ax2.hist([obj_x, tgt_x], bins=20, alpha=0.7, label=['Objects', 'Targets'], color=['blue', 'orange'])
    ax2.set_xlabel('X Position (m)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('X-Position Distribution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # Y-position distribution
    ax3 = fig.add_subplot(2, 2, 3)
    obj_y = [pos[1] for pos in obj_positions]
    tgt_y = [pos[1] for pos in tgt_positions]
    
    ax3.hist([obj_y, tgt_y], bins=20, alpha=0.7, label=['Objects', 'Targets'], color=['blue', 'orange'])
    ax3.set_xlabel('Y Position (m)')
    ax3.set_ylabel('Frequency')
    ax3.set_title('Y-Position Distribution')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Distance distribution
    ax4 = fig.add_subplot(2, 2, 4)
    distances = [np.sqrt((obj[0] - tgt[0])**2 + (obj[1] - tgt[1])**2) 
                for obj, tgt in zip(obj_positions, tgt_positions)]
    
    success_distances = [dist for dist, success in zip(distances, successes) if success]
    fail_distances = [dist for dist, success in zip(distances, successes) if not success]
    
    if success_distances:
        ax4.hist(success_distances, bins=15, alpha=0.7, color='green', label='Successful')
    if fail_distances:
        ax4.hist(fail_distances, bins=15, alpha=0.7, color='red', label='Failed')
    
    ax4.set_xlabel('Object-Target Distance (m)')
    ax4.set_ylabel('Frequency')
    ax4.set_title('Distance Distribution by Success')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def create_benchmark_color_visualization(object_colors, target_colors, cases, benchmark_name):
    """Create color distribution visualization for benchmark results."""
    from collections import Counter
    
    # Extract success information
    successes = [case.get('success', False) for case in cases]
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # Overall color distribution
    if object_colors:
        obj_counts = Counter(object_colors)
        colors_obj = list(obj_counts.keys())
        counts_obj = list(obj_counts.values())
        
        bars1 = ax1.bar(colors_obj, counts_obj, alpha=0.8, color='skyblue')
        ax1.set_xlabel('Object Colors')
        ax1.set_ylabel('Frequency')
        ax1.set_title('Overall Object Color Distribution')
        ax1.tick_params(axis='x', rotation=45)
        ax1.grid(True, alpha=0.3)
        
        for bar, count in zip(bars1, counts_obj):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    str(count), ha='center', va='bottom')
    
    if target_colors:
        tgt_counts = Counter(target_colors)
        colors_tgt = list(tgt_counts.keys())
        counts_tgt = list(tgt_counts.values())
        
        bars2 = ax2.bar(colors_tgt, counts_tgt, alpha=0.8, color='orange')
        ax2.set_xlabel('Target Colors')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Overall Target Color Distribution')
        ax2.tick_params(axis='x', rotation=45)
        ax2.grid(True, alpha=0.3)
        
        for bar, count in zip(bars2, counts_tgt):
            ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                    str(count), ha='center', va='bottom')
    
    # Success/failure by color
    if object_colors and successes:
        success_obj_colors = [color for color, success in zip(object_colors, successes) if success]
        fail_obj_colors = [color for color, success in zip(object_colors, successes) if not success]
        
        success_counts = Counter(success_obj_colors)
        fail_counts = Counter(fail_obj_colors)
        
        all_obj_colors = set(success_counts.keys()) | set(fail_counts.keys())
        
        success_vals = [success_counts.get(color, 0) for color in all_obj_colors]
        fail_vals = [fail_counts.get(color, 0) for color in all_obj_colors]
        
        x = np.arange(len(all_obj_colors))
        width = 0.35
        
        bars3 = ax3.bar(x - width/2, success_vals, width, label='Successful', color='green', alpha=0.8)
        bars4 = ax3.bar(x + width/2, fail_vals, width, label='Failed', color='red', alpha=0.8)
        
        ax3.set_xlabel('Object Colors')
        ax3.set_ylabel('Frequency')
        ax3.set_title('Object Color Success vs Failure')
        ax3.set_xticks(x)
        ax3.set_xticklabels(all_obj_colors, rotation=45)
        ax3.legend()
        ax3.grid(True, alpha=0.3)
    
    # Success rate by color
    if object_colors and successes:
        color_success_rates = {}
        for color in set(object_colors):
            color_cases = [success for color_case, success in zip(object_colors, successes) if color_case == color]
            if color_cases:
                success_rate = (sum(color_cases) / len(color_cases)) * 100
                color_success_rates[color] = success_rate
        
        if color_success_rates:
            colors_rate = list(color_success_rates.keys())
            rates = list(color_success_rates.values())
            
            bars5 = ax4.bar(colors_rate, rates, alpha=0.8, color='purple')
            ax4.set_xlabel('Object Colors')
            ax4.set_ylabel('Success Rate (%)')
            ax4.set_title('Success Rate by Object Color')
            ax4.tick_params(axis='x', rotation=45)
            ax4.grid(True, alpha=0.3)
            ax4.set_ylim(0, 100)
            
            for bar, rate in zip(bars5, rates):
                ax4.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                        f'{rate:.1f}%', ha='center', va='bottom')
    
    plt.tight_layout()
    return fig

def create_benchmark_performance_analysis(cases, benchmark_name):
    """Create performance analysis visualization."""
    if not cases:
        return None
    
    try:
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # Success rate by completion time
        successful_cases = [case for case in cases if case.get('success', False)]
        completion_times = [case.get('completion_time', 0) for case in successful_cases if case.get('completion_time')]
        
        if completion_times:
            ax1.hist(completion_times, bins=20, alpha=0.7, color='green')
            ax1.set_xlabel('Completion Time (s)')
            ax1.set_ylabel('Frequency')
            ax1.set_title('Completion Time Distribution (Successful Cases)')
            ax1.grid(True, alpha=0.3)
        
        # Success rate by distance
        distances = []
        successes = []
        
        for case in cases:
            if 'object' in case and 'target' in case:
                obj_pos = case['object']['pos']
                tgt_pos = case['target']['pos']
                dist = np.sqrt((obj_pos[0] - tgt_pos[0])**2 + (obj_pos[1] - tgt_pos[1])**2)
                distances.append(dist)
                successes.append(case.get('success', False))
        
        if distances:
            # Bin distances and calculate success rate per bin
            bins = np.linspace(min(distances), max(distances), 10)
            bin_centers = (bins[:-1] + bins[1:]) / 2
            success_rates = []
            
            for i in range(len(bins) - 1):
                bin_mask = (np.array(distances) >= bins[i]) & (np.array(distances) < bins[i+1])
                bin_successes = np.array(successes)[bin_mask]
                if len(bin_successes) > 0:
                    success_rate = (sum(bin_successes) / len(bin_successes)) * 100
                else:
                    success_rate = 0
                success_rates.append(success_rate)
            
            ax2.bar(bin_centers, success_rates, width=bins[1]-bins[0], alpha=0.7, color='blue')
            ax2.set_xlabel('Object-Target Distance (m)')
            ax2.set_ylabel('Success Rate (%)')
            ax2.set_title('Success Rate by Distance')
            ax2.grid(True, alpha=0.3)
            ax2.set_ylim(0, 100)
        
        # Individual metric performance
        if 'metrics' in cases[0]:
            metrics = list(cases[0]['metrics'].keys())
            metric_success_rates = []
            
            for metric in metrics:
                metric_successes = [case['metrics'][metric] for case in cases if 'metrics' in case and metric in case['metrics']]
                if metric_successes:
                    rate = (sum(metric_successes) / len(metric_successes)) * 100
                    metric_success_rates.append(rate)
                else:
                    metric_success_rates.append(0)
            
            bars = ax3.bar([m.replace('_', '\n') for m in metrics], metric_success_rates, alpha=0.8)
            ax3.set_ylabel('Success Rate (%)')
            ax3.set_title('Individual Metric Performance')
            ax3.tick_params(axis='x', rotation=45)
            ax3.grid(True, alpha=0.3)
            ax3.set_ylim(0, 100)
            
            # Add value labels
            for bar, rate in zip(bars, metric_success_rates):
                ax3.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                        f'{rate:.1f}%', ha='center', va='bottom')
        
        # Overall statistics
        total_cases = len(cases)
        successful = sum(1 for case in cases if case.get('success', False))
        success_rate = (successful / total_cases) * 100 if total_cases > 0 else 0
        
        ax4.pie([successful, total_cases - successful], 
               labels=['Successful', 'Failed'], 
               colors=['green', 'red'], 
               autopct='%1.1f%%',
               startangle=90)
        ax4.set_title(f'Overall Success Rate\n({successful}/{total_cases} cases)')
        
        plt.suptitle(f'Performance Analysis - {benchmark_name}', fontsize=16, fontweight='bold')
        plt.tight_layout()
        
        return fig
        
    except Exception as e:
        print(f"Error creating performance analysis: {e}")
        return None

def main():
    st.markdown('<div class="main-header">🤖 Isaac Lab Experiment Dashboard</div>', unsafe_allow_html=True)
    
    # Sidebar for navigation and configuration
    st.sidebar.markdown("## 🎛️ Navigation")
    
    # Base directory selection
    default_base_dirs = [
        "/home/innovation-hacking/luebbet/models/pipeline",
        "/home/innovation-hacking/luebbet/experiments/pipeline"
    ]
    
    base_dir_options = ["Custom Path..."] + [d for d in default_base_dirs if os.path.exists(d)]
    selected_base = st.sidebar.selectbox("Base Experiment Directory:", base_dir_options)
    
    if selected_base == "Custom Path...":
        base_dir = st.sidebar.text_input("Enter custom path:", "/home/innovation-hacking/luebbet/models/pipeline")
    else:
        base_dir = selected_base
    
    # Scan experiment directories
    if base_dir and os.path.exists(base_dir):
        experiment_dirs = scan_experiment_directories(base_dir)
        
        if not experiment_dirs:
            st.warning(f"No experiment directories found in {base_dir}")
            return
            
        st.sidebar.markdown(f"**Found {len(experiment_dirs)} experiments**")
        
        # Main dashboard tabs
        tabs = st.tabs([
            "📊 Overview", 
            "🔄 Compare Experiments", 
            "📋 Dataset Analysis", 
            "📉 TensorBoard Logs",
            "� Benchmark Summary"
        ])
        
        with tabs[0]:  # Overview
            show_overview(experiment_dirs, base_dir)
            
        with tabs[1]:  # Compare Experiments
            show_experiment_comparison(experiment_dirs)
            
        with tabs[2]:  # Dataset Analysis
            show_dataset_analysis(experiment_dirs)
            
        with tabs[3]:  # TensorBoard Logs
            show_tensorboard_analysis(experiment_dirs)
            
        with tabs[4]:  # Benchmark Summary
            show_benchmark_results(experiment_dirs)
    
    else:
        st.error(f"Directory not found: {base_dir}")

def show_overview(experiment_dirs, base_dir):
    """Show overview of all experiments."""
    st.markdown('<div class="section-header">🎯 Experiment Overview</div>', unsafe_allow_html=True)
    
    # Summary statistics
    col1, col2, col3, col4 = st.columns(4)
    
    total_experiments = len(experiment_dirs)
    experiments_with_models = sum(1 for exp_dir in experiment_dirs if get_experiment_info(exp_dir)['models'])
    experiments_with_benchmarks = sum(1 for exp_dir in experiment_dirs if get_experiment_info(exp_dir)['benchmarks'])
    experiments_with_tensorboard = sum(1 for exp_dir in experiment_dirs if get_experiment_info(exp_dir)['has_tensorboard'])
    
    col1.metric("Total Experiments", total_experiments)
    col2.metric("With Models", experiments_with_models)
    col3.metric("With Benchmarks", experiments_with_benchmarks) 
    col4.metric("With TensorBoard", experiments_with_tensorboard)
    
    st.markdown("---")
    
    # Experiment details table
    st.markdown("### 📋 Experiment Details")
    
    experiment_data = []
    for exp_dir in experiment_dirs:
        info = get_experiment_info(exp_dir)
        experiment_data.append({
            "Name": info['name'],
            "Models": len(info['models']),
            "Benchmarks": len(info['benchmarks']),
            "Datasets": len(info['datasets']),
            "TensorBoard": "✅" if info['has_tensorboard'] else "❌",
            "Data Viz": "✅" if info['has_data_viz'] else "❌",
            "Path": info['path']
        })
    
    df = pd.DataFrame(experiment_data)
    st.dataframe(df, use_container_width=True)
    
    # Detailed experiment cards
    st.markdown("### 🔍 Detailed View")
    
    selected_experiments = st.multiselect(
        "Select experiments to view details:",
        options=[os.path.basename(exp_dir) for exp_dir in experiment_dirs],
        default=[]
    )
    
    for exp_name in selected_experiments:
        exp_dir = next(exp_dir for exp_dir in experiment_dirs if os.path.basename(exp_dir) == exp_name)
        info = get_experiment_info(exp_dir)
        
        with st.expander(f"📁 {exp_name}", expanded=True):
            col1, col2 = st.columns(2)
            
            with col1:
                st.markdown("**Models:**")
                for model in info['models']:
                    st.write(f"- {model}")
                
                st.markdown("**Benchmarks:**")
                for benchmark in info['benchmarks']:
                    st.write(f"- {benchmark}")
            
            with col2:
                st.markdown("**Datasets:**")
                for dataset in info['datasets']:
                    st.write(f"- {dataset}")
                
                st.markdown("**Features:**")
                st.write(f"- TensorBoard: {'✅' if info['has_tensorboard'] else '❌'}")
                st.write(f"- Data Visualization: {'✅' if info['has_data_viz'] else '❌'}")
            
            st.markdown(f"**Path:** `{info['path']}`")

def show_experiment_comparison(experiment_dirs):
    """Show experiment comparison analysis."""
    st.markdown('<div class="section-header">🔄 Experiment Comparison</div>', unsafe_allow_html=True)
    
    # Select benchmark
    available_benchmarks = get_available_benchmarks()
    if not available_benchmarks:
        st.error("No benchmarks found")
        return
    
    selected_benchmark = st.selectbox("Select Benchmark:", available_benchmarks, key="comparison_benchmark")
    
    # Select experiments to compare
    valid_experiments = []
    for exp_dir in experiment_dirs:
        info = get_experiment_info(exp_dir)
        if selected_benchmark in info['benchmarks']:
            valid_experiments.append(exp_dir)
    
    if not valid_experiments:
        st.warning(f"No experiments found with results for benchmark: {selected_benchmark}")
        return
    
    selected_experiments = st.multiselect(
        "Select experiments to compare:",
        options=[os.path.basename(exp_dir) for exp_dir in valid_experiments],
        default=[os.path.basename(exp_dir) for exp_dir in valid_experiments[:3]],  # Select first 3 by default
        key="comparison_experiments"
    )
    
    if len(selected_experiments) < 2:
        st.warning("Please select at least 2 experiments to compare")
        return
    
    # Convert back to full paths
    selected_exp_dirs = [
        next(exp_dir for exp_dir in valid_experiments if os.path.basename(exp_dir) == exp_name)
        for exp_name in selected_experiments
    ]
    
    # Generate comparison
    if st.button("🔄 Compare Experiments", type="primary"):
        with st.spinner("Comparing experiments..."):
            try:
                # Collect experiment data
                experiments_data = collect_comparison_data(selected_exp_dirs, selected_benchmark)
                
                if not experiments_data:
                    st.error("No valid experiment data found")
                    return
                
                # Create summary table
                summary_table, num_runs = create_comparison_summary_table(experiments_data)
                
                # Display results
                st.success(f"✅ Compared {len(experiments_data)} experiments")
                
                # Show comparison plots
                col1, col2 = st.columns(2)
                
                with col1:
                    st.markdown("#### Overall Success Comparison")
                    fig1, ax1 = plt.subplots(figsize=(12, 6))
                    
                    experiment_names = [exp['experiment_name'] for exp in experiments_data]
                    success_means = [exp['overall_success_mean'] for exp in experiments_data]
                    success_stds = [exp['overall_success_std'] for exp in experiments_data]
                    
                    x_pos = np.arange(len(experiment_names))
                    bars = ax1.bar(x_pos, success_means, yerr=success_stds, 
                                  capsize=5, color='steelblue', alpha=0.8)
                    
                    ax1.set_xlabel('Experiments')
                    ax1.set_ylabel('Overall Success Rate (%)')
                    ax1.set_title(f'Success Rate Comparison: {selected_benchmark}')
                    ax1.set_xticks(x_pos)
                    ax1.set_xticklabels([name.replace('_', '\n') for name in experiment_names], rotation=0, ha='center')
                    ax1.grid(True, alpha=0.3)
                    
                    # Add value labels on bars
                    for bar, mean_val, std_val in zip(bars, success_means, success_stds):
                        height = bar.get_height()
                        ax1.annotate(f'{mean_val:.1f}±{std_val:.1f}%',
                                   xy=(bar.get_x() + bar.get_width() / 2, height + std_val),
                                   xytext=(0, 3), textcoords="offset points",
                                   ha='center', va='bottom', fontsize=8, fontweight='bold')
                    
                    st.pyplot(fig1)
                    create_matplotlib_figure_download(fig1, f"comparison_success_{selected_benchmark}")
                
                with col2:
                    st.markdown("#### Metrics Comparison")
                    
                    # Get all unique metrics
                    all_metrics = set()
                    for exp in experiments_data:
                        all_metrics.update(exp['metric_rates_mean'].keys())
                    
                    if all_metrics:
                        fig2, ax2 = plt.subplots(figsize=(12, 6))
                        
                        n_metrics = len(all_metrics)
                        n_experiments = len(experiments_data)
                        bar_width = 0.8 / n_metrics
                        colors = plt.cm.get_cmap('tab10')(np.linspace(0, 1, n_metrics))
                        
                        x_pos = np.arange(n_experiments)
                        
                        for i, metric in enumerate(sorted(all_metrics)):
                            metric_means = []
                            metric_stds = []
                            
                            for exp in experiments_data:
                                mean_val = exp['metric_rates_mean'].get(metric, 0)
                                individual_values = exp.get('metric_rates_all', {}).get(metric, [mean_val])
                                std_val = np.std(individual_values) if len(individual_values) > 1 else 0
                                
                                metric_means.append(mean_val)
                                metric_stds.append(std_val)
                            
                            offset_x = x_pos + i * bar_width - (n_metrics - 1) * bar_width / 2
                            ax2.bar(offset_x, metric_means, bar_width, 
                                   yerr=metric_stds, capsize=3,
                                   label=metric.replace('_', ' ').title(),
                                   color=colors[i], alpha=0.8)
                        
                        ax2.set_xlabel('Experiments')
                        ax2.set_ylabel('Success Rate (%)')
                        ax2.set_title('Individual Metrics Comparison')
                        ax2.set_xticks(x_pos)
                        ax2.set_xticklabels([name.replace('_', '\n') for name in experiment_names], rotation=0, ha='center')
                        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
                        ax2.grid(True, alpha=0.3)
                        
                        st.pyplot(fig2)
                        create_matplotlib_figure_download(fig2, f"comparison_metrics_{selected_benchmark}")
                    else:
                        st.info("No individual metrics found")
                
                # Summary statistics
                st.markdown("#### Comparison Summary")
                
                best_exp = max(experiments_data, key=lambda x: x['overall_success_mean'])
                worst_exp = min(experiments_data, key=lambda x: x['overall_success_mean'])
                
                col1, col2, col3 = st.columns(3)
                col1.metric("Best Performance", f"{best_exp['experiment_name']}", f"{best_exp['overall_success_mean']:.1f}%")
                col2.metric("Worst Performance", f"{worst_exp['experiment_name']}", f"{worst_exp['overall_success_mean']:.1f}%")
                col3.metric("Performance Gap", "", f"{best_exp['overall_success_mean'] - worst_exp['overall_success_mean']:.1f}%")
                
                # Summary table
                st.markdown("#### Detailed Results Table")
                st.dataframe(pd.DataFrame(summary_table), use_container_width=True)
                
            except Exception as e:
                st.error(f"Error comparing experiments: {str(e)}")

def show_dataset_analysis(experiment_dirs):
    """Show dataset distribution analysis using the standard visualize_dataset_distr.py output."""
    st.markdown('<div class="section-header">📋 Dataset Distribution Analysis</div>', unsafe_allow_html=True)
    
    # Select experiment with data visualization
    experiments_with_data_viz = []
    for exp_dir in experiment_dirs:
        info = get_experiment_info(exp_dir)
        if info['has_data_viz']:
            experiments_with_data_viz.append(exp_dir)
    
    if not experiments_with_data_viz:
        st.warning("No experiments found with dataset visualization files")
        return
    
    selected_exp = st.selectbox(
        "Select Experiment:",
        options=[os.path.basename(exp_dir) for exp_dir in experiments_with_data_viz],
        key="dataset_analysis_exp"
    )
    
    selected_exp_dir = next(exp_dir for exp_dir in experiments_with_data_viz if os.path.basename(exp_dir) == selected_exp)
    
    # Get available datasets
    data_viz_dir = os.path.join(selected_exp_dir, 'data_visualization')
    yaml_files = glob.glob(os.path.join(data_viz_dir, '*.yaml'))
    
    if not yaml_files:
        st.warning("No dataset YAML files found")
        return
    
    dataset_names = [os.path.basename(f).replace('.yaml', '') for f in yaml_files]
    selected_dataset = st.selectbox("Select Dataset:", dataset_names, key="dataset_analysis_dataset")
    
    dataset_yaml = os.path.join(data_viz_dir, f"{selected_dataset}.yaml")
    
    # Additional analysis options
    st.markdown("### Additional Analysis Options")
    
    col1, col2 = st.columns(2)
    show_camera_poses = col1.checkbox("Show Camera Pose Distribution", False)
    show_color_distribution = col2.checkbox("Show Color Distribution", False)
    
    # Show camera pose analysis if selected
    if show_camera_poses:
        st.subheader("📹 Camera Pose Distribution")
        
        # Find dataset YAML file for the selected dataset
        yaml_pattern = f"/home/innovation-hacking/luebbet/dev/IsaacLab/tng_datasets/{selected_dataset}*.yaml"
        yaml_files = glob.glob(yaml_pattern)
        
        if yaml_files:
            try:
                yaml_path = yaml_files[0]
                camera_data = extract_camera_poses(yaml_path)
                
                # Check if we have any camera data
                has_data = any(data['positions'] or data['rotations'] 
                             for data in camera_data.values())
                
                if has_data:
                    with st.spinner("Generating camera pose visualization..."):
                        fig = create_camera_pose_visualization(camera_data, selected_dataset)
                        st.pyplot(fig)
                        create_matplotlib_figure_download(fig, f"camera_poses_{selected_dataset}")
                        plt.close(fig)
                else:
                    st.warning("No camera pose data found in dataset")
                    
            except Exception as e:
                st.error(f"Error analyzing camera poses: {str(e)}")
        else:
            st.warning(f"No dataset YAML file found for {selected_dataset}")
    
    # Show color distribution analysis if selected
    if show_color_distribution:
        st.subheader("🎨 Color Distribution")
        
        # Find dataset YAML file for the selected dataset
        yaml_pattern = f"/home/innovation-hacking/luebbet/dev/IsaacLab/tng_datasets/{selected_dataset}*.yaml"
        yaml_files = glob.glob(yaml_pattern)
        
        if yaml_files:
            try:
                yaml_path = yaml_files[0]
                object_colors, target_colors = extract_colors(yaml_path)
                
                if object_colors or target_colors:
                    with st.spinner("Generating color distribution visualization..."):
                        fig = create_color_distribution_visualization(
                            object_colors, target_colors, selected_dataset)
                        st.pyplot(fig)
                        create_matplotlib_figure_download(fig, f"color_distribution_{selected_dataset}")
                        plt.close(fig)
                        
                    # Show statistics
                    col1, col2 = st.columns(2)
                    with col1:
                        st.metric("Unique Object Colors", len(set(object_colors)) if object_colors else 0)
                    with col2:
                        st.metric("Unique Target Colors", len(set(target_colors)) if target_colors else 0)
                else:
                    st.warning("No color data found in dataset")
                    
            except Exception as e:
                st.error(f"Error analyzing color distribution: {str(e)}")
        else:
            st.warning(f"No dataset YAML file found for {selected_dataset}")
    
    # Generate visualization
    if st.button("📊 Generate Dataset Visualization", type="primary"):
        with st.spinner("Generating dataset visualization..."):
            try:
                # Import and use the visualize_dataset_distr main function
                import subprocess
                import tempfile
                
                # Create a temporary output directory for the visualization
                with tempfile.TemporaryDirectory() as temp_dir:
                    # Run the visualize_dataset_distr.py script to generate the standard combined analysis
                    cmd = [
                        "/home/innovation-hacking/luebbet/venvs/isaaclab_luebbet/bin/python",
                        "/home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils/visualize_dataset_distr.py",
                        "--in", dataset_yaml,
                        "--out-dir", temp_dir
                    ]
                    
                    result = subprocess.run(cmd, capture_output=True, text=True, cwd="/home/innovation-hacking/luebbet/dev/IsaacLab/scripts/tng/utils")
                    
                    if result.returncode != 0:
                        st.error(f"Error generating visualization: {result.stderr}")
                        return
                    
                    # Look for the generated combined analysis image
                    generated_files = glob.glob(os.path.join(temp_dir, "*_combined_analysis*.png"))
                    
                    if not generated_files:
                        st.error("No visualization files were generated")
                        return
                    
                    # Display the generated visualization
                    for viz_file in generated_files:
                        st.success("✅ Generated dataset visualization")
                        
                        # Display the image
                        st.image(viz_file, caption=f"Dataset Analysis: {selected_dataset}")
                        
                        # Provide download link
                        with open(viz_file, "rb") as f:
                            st.download_button(
                                label="📥 Download Visualization",
                                data=f.read(),
                                file_name=f"dataset_analysis_{selected_dataset}_{selected_exp}.png",
                                mime="image/png"
                            )
                        break
                    
                    # Show basic statistics
                    st.markdown("#### Dataset Statistics")
                    
                    # Load dataset for basic stats
                    obj_pts, tgt_pts, lines, obj_yaws, tgt_yaws, table_heights, joint_offsets, gripper_offsets, metadata = load_positions(dataset_yaml)
                    
                    if obj_pts:
                        col1, col2, col3, col4 = st.columns(4)
                        
                        # Calculate basic statistics
                        distances = [np.sqrt((line[1][0] - line[0][0])**2 + (line[1][1] - line[0][1])**2) for line in lines]
                        
                        col1.metric("Total Cases", len(obj_pts))
                        col2.metric("Unique Object Positions", len(set(obj_pts)))
                        col3.metric("Unique Target Positions", len(set(tgt_pts)))
                        col4.metric("Avg Travel Distance", f"{np.mean(distances):.3f}m")
                        
                        # Additional statistics
                        st.markdown("#### Additional Information")
                        
                        info_cols = st.columns(3)
                        info_cols[0].metric("Min Distance", f"{np.min(distances):.3f}m")
                        info_cols[1].metric("Max Distance", f"{np.max(distances):.3f}m")
                        info_cols[2].metric("Distance Std Dev", f"{np.std(distances):.3f}m")
                        
                        # Enhanced data indicators
                        if obj_yaws and any(yaw != 0 for yaw in obj_yaws):
                            st.info("✨ Dataset includes object rotation information (yaw angles)")
                        
                        if table_heights and any(h != 0 for h in table_heights):
                            st.info("📏 Dataset includes table height variations")
                        
                        if joint_offsets and any(any(offset != 0 for offset in offsets) for offsets in joint_offsets):
                            st.info("🦾 Dataset includes robot joint offset variations")
                    
            except Exception as e:
                st.error(f"Error generating dataset visualization: {str(e)}")
                st.error("Make sure the visualize_dataset_distr.py script is available and working correctly.")

def show_tensorboard_analysis(experiment_dirs):
    """Show TensorBoard log analysis."""
    st.markdown('<div class="section-header">📉 TensorBoard Log Analysis</div>', unsafe_allow_html=True)
    
    # Select experiments with TensorBoard logs
    experiments_with_tb = []
    for exp_dir in experiment_dirs:
        info = get_experiment_info(exp_dir)
        if info['has_tensorboard']:
            experiments_with_tb.append(exp_dir)
    
    if not experiments_with_tb:
        st.warning("No experiments found with TensorBoard logs")
        return
    
    selected_exp = st.selectbox(
        "Select Experiment:",
        options=[os.path.basename(exp_dir) for exp_dir in experiments_with_tb],
        key="tb_experiment"
    )
    
    selected_exp_dir = next(exp_dir for exp_dir in experiments_with_tb if os.path.basename(exp_dir) == selected_exp)
    
    # Find TensorBoard log directories
    tb_dirs = []
    for item in os.listdir(selected_exp_dir):
        item_path = os.path.join(selected_exp_dir, item)
        if os.path.isdir(item_path) and item.startswith('models_'):
            # Look for subdirectories with TensorBoard logs
            for subitem in os.listdir(item_path):
                subitem_path = os.path.join(item_path, subitem)
                if os.path.isdir(subitem_path):
                    tb_files = glob.glob(os.path.join(subitem_path, '**/events.out.tfevents.*'), recursive=True)
                    if tb_files:
                        tb_dirs.append(subitem_path)
    
    if not tb_dirs:
        st.error("No TensorBoard log directories found")
        return
    
    selected_tb_dir = st.selectbox("Select Model/Checkpoint:", [os.path.basename(d) for d in tb_dirs], key="tb_model")
    selected_tb_path = next(d for d in tb_dirs if os.path.basename(d) == selected_tb_dir)
    
    # Analysis options
    st.markdown("### Analysis Options")
    
    col1, col2, col3 = st.columns(3)
    loss_tag = col1.text_input("Loss Tag:", "train/loss")
    lr_tag = col2.text_input("Learning Rate Tag:", "train/learning_rate") 
    smooth_span = col3.number_input("Smoothing Span:", min_value=1, max_value=1000, value=100)
    
    # Load and analyze logs
    if st.button("📊 Analyze TensorBoard Logs", type="primary"):
        with st.spinner("Loading TensorBoard logs..."):
            try:
                # Load scalars
                df = load_scalars(selected_tb_path)
                
                if df.empty:
                    st.error("No TensorBoard data found")
                    return
                
                st.success(f"✅ Loaded TensorBoard data: {len(df)} records")
                
                # Show available tags
                available_tags = sorted(df['tag'].unique())
                st.markdown("#### Available Tags")
                st.write(", ".join(available_tags))
                
                # Generate summary
                try:
                    summary = summarize_runs(df, loss_tag, lr_tag, smooth_span)
                    
                    st.markdown("#### Training Summary")
                    if isinstance(summary, pd.DataFrame) and not summary.empty:
                        st.dataframe(summary, use_container_width=True)
                    else:
                        st.warning("Could not generate training summary. Check tag names.")
                        
                except Exception as e:
                    st.warning(f"Could not generate summary: {str(e)}")
                
                # Plot training curves
                st.markdown("#### Training Curves")
                
                # Filter data for plotting
                loss_data = df[df['tag'] == loss_tag]
                lr_data = df[df['tag'] == lr_tag]
                
                if not loss_data.empty:
                    col1, col2 = st.columns(2)
                    
                    with col1:
                        st.markdown("**Loss Curve**")
                        fig1, ax1 = plt.subplots(figsize=(10, 6))
                        
                        # Plot loss for each run
                        runs = loss_data['run'].unique()
                        colors = plt.cm.get_cmap('tab10')(np.linspace(0, 1, len(runs)))
                        
                        for i, run in enumerate(runs):
                            run_data = loss_data[loss_data['run'] == run]
                            
                            # Plot raw data
                            ax1.plot(run_data['step'], run_data['value'], 
                                   alpha=0.3, color=colors[i], linewidth=1)
                            
                            # Plot smoothed data if enough points
                            if len(run_data) > smooth_span:
                                smoothed = run_data['value'].ewm(span=smooth_span, adjust=False).mean()
                                ax1.plot(run_data['step'], smoothed, 
                                       color=colors[i], linewidth=2, label=f'Run {run}')
                        
                        ax1.set_xlabel('Training Step')
                        ax1.set_ylabel('Loss')
                        ax1.set_title(f'Training Loss: {loss_tag}')
                        ax1.grid(True, alpha=0.3)
                        ax1.legend()
                        
                        st.pyplot(fig1)
                        create_matplotlib_figure_download(fig1, f"loss_curve_{selected_exp}")
                    
                    with col2:
                        if not lr_data.empty:
                            st.markdown("**Learning Rate**")
                            fig2, ax2 = plt.subplots(figsize=(10, 6))
                            
                            # Plot learning rate for each run
                            for i, run in enumerate(runs):
                                run_data = lr_data[lr_data['run'] == run]
                                if not run_data.empty:
                                    ax2.plot(run_data['step'], run_data['value'], 
                                           color=colors[i], linewidth=2, label=f'Run {run}')
                            
                            ax2.set_xlabel('Training Step')
                            ax2.set_ylabel('Learning Rate')
                            ax2.set_title(f'Learning Rate: {lr_tag}')
                            ax2.grid(True, alpha=0.3)
                            ax2.legend()
                            
                            st.pyplot(fig2)
                            create_matplotlib_figure_download(fig2, f"lr_curve_{selected_exp}")
                        else:
                            st.info("No learning rate data found")
                
                else:
                    st.warning(f"No data found for loss tag: {loss_tag}")
                
                # Show other metrics if available
                other_tags = [tag for tag in available_tags if tag not in [loss_tag, lr_tag]]
                
                if other_tags:
                    st.markdown("#### Other Metrics")
                    
                    selected_other_tags = st.multiselect(
                        "Select additional metrics to plot:",
                        options=other_tags,
                        default=other_tags[:3] if len(other_tags) >= 3 else other_tags
                    )
                    
                    if selected_other_tags:
                        n_plots = len(selected_other_tags)
                        n_cols = min(2, n_plots)
                        n_rows = (n_plots + n_cols - 1) // n_cols
                        
                        fig, axes = plt.subplots(n_rows, n_cols, figsize=(12, 4*n_rows))
                        if n_plots == 1:
                            axes = [axes]
                        elif n_rows == 1:
                            axes = [axes]
                        else:
                            axes = axes.flatten()
                        
                        for i, tag in enumerate(selected_other_tags):
                            if i < len(axes):
                                ax = axes[i]
                                tag_data = df[df['tag'] == tag]
                                
                                if not tag_data.empty:
                                    runs = tag_data['run'].unique()
                                    colors = plt.cm.get_cmap('tab10')(np.linspace(0, 1, len(runs)))
                                    
                                    for j, run in enumerate(runs):
                                        run_data = tag_data[tag_data['run'] == run]
                                        ax.plot(run_data['step'], run_data['value'], 
                                              color=colors[j], linewidth=2, label=f'Run {run}')
                                    
                                    ax.set_xlabel('Training Step')
                                    ax.set_ylabel(tag.split('/')[-1])
                                    ax.set_title(tag)
                                    ax.grid(True, alpha=0.3)
                                    if len(runs) > 1:
                                        ax.legend()
                        
                        # Hide empty subplots
                        for i in range(len(selected_other_tags), len(axes)):
                            axes[i].set_visible(False)
                        
                        plt.tight_layout()
                        st.pyplot(fig)
                        create_matplotlib_figure_download(fig, f"other_metrics_{selected_exp}")
                
            except Exception as e:
                st.error(f"Error analyzing TensorBoard logs: {str(e)}")

def show_benchmark_results(experiment_dirs):
    """Show benchmark result analysis."""
    st.markdown('<div class="section-header">🎯 Benchmark Result Analysis</div>', unsafe_allow_html=True)
    
    # Select experiment
    experiments_with_benchmarks = []
    for exp_dir in experiment_dirs:
        info = get_experiment_info(exp_dir)
        if info['benchmarks']:
            experiments_with_benchmarks.append(exp_dir)
    
    if not experiments_with_benchmarks:
        st.warning("No experiments found with benchmark results")
        return
    
    selected_exp = st.selectbox(
        "Select Experiment:",
        options=[os.path.basename(exp_dir) for exp_dir in experiments_with_benchmarks],
        key="benchmark_results_experiment"
    )
    
    selected_exp_dir = next(exp_dir for exp_dir in experiments_with_benchmarks if os.path.basename(exp_dir) == selected_exp)
    
    # Select benchmark
    info = get_experiment_info(selected_exp_dir)
    selected_benchmark = st.selectbox("Select Benchmark:", info['benchmarks'], key="benchmark_results_benchmark")
    
    # Generate benchmark visualization
    if st.button("🎯 Generate Benchmark Analysis", type="primary"):
        with st.spinner("Analyzing benchmark results..."):
            try:
                # Use the existing benchmark analysis function
                create_benchmark_visualization(selected_exp_dir, selected_benchmark)
                
                # Display the generated plot
                plot_path = os.path.join(selected_exp_dir, f"benchmark_analysis_{selected_benchmark}.png")
                
                if os.path.exists(plot_path):
                    st.success("✅ Generated benchmark analysis")
                    st.image(plot_path, caption=f"Benchmark Analysis: {selected_benchmark}")
                    
                    # Provide download link
                    with open(plot_path, "rb") as f:
                        st.download_button(
                            label="📥 Download Analysis",
                            data=f.read(),
                            file_name=f"benchmark_analysis_{selected_benchmark}_{selected_exp}.png",
                            mime="image/png"
                        )
                else:
                    st.warning("Benchmark analysis completed but plot not found at expected location")
                
                # Show summary statistics
                st.markdown("#### Benchmark Summary")
                
                # Collect benchmark results for summary
                case_results = collect_benchmark_results(selected_exp_dir, selected_benchmark)
                
                if case_results:
                    # Calculate basic statistics
                    total_cases = len(case_results)
                    # Count unique runs by collecting all unique run names
                    unique_runs = set()
                    for results in case_results.values():
                        for result in results:
                            unique_runs.add(result['run'])
                    total_runs = len(unique_runs)
                    
                    # Calculate overall success rate
                    all_successes = []
                    for results in case_results.values():
                        for result in results:
                            all_successes.append(result['success'])
                    
                    overall_success_rate = np.mean(all_successes) * 100
                    success_std = np.std(all_successes) * 100
                    
                    col1, col2, col3, col4 = st.columns(4)
                    col1.metric("Total Test Cases", total_cases)
                    col2.metric("Total Runs", total_runs)
                    col3.metric("Overall Success Rate", f"{overall_success_rate:.1f}%")
                    col4.metric("Success Std Dev", f"{success_std:.1f}%")
                    
                    # Show per-case success rates
                    st.markdown("#### Per-Case Success Rates")
                    
                    case_stats = []
                    for case_id, results in case_results.items():
                        case_successes = [r['success'] for r in results]
                        case_success_rate = np.mean(case_successes) * 100
                        case_std = np.std(case_successes) * 100 if len(case_successes) > 1 else 0
                        
                        case_stats.append({
                            "Case ID": case_id,
                            "Runs": len(results),
                            "Success Rate (%)": f"{case_success_rate:.1f}",
                            "Std Dev (%)": f"{case_std:.1f}",
                            "Object Pos": f"({results[0]['object_pos'][0]:.3f}, {results[0]['object_pos'][1]:.3f})",
                            "Target Pos": f"({results[0]['target_pos'][0]:.3f}, {results[0]['target_pos'][1]:.3f})"
                        })
                    
                    # Sort by success rate (descending)
                    case_stats.sort(key=lambda x: float(x["Success Rate (%)"].rstrip('%')), reverse=True)
                    
                    st.dataframe(pd.DataFrame(case_stats), use_container_width=True)
                
            except Exception as e:
                st.error(f"Error analyzing benchmark results: {str(e)}")

if __name__ == "__main__":
    main()