# make_figs.py
# Usage:
#   python make_figs.py --logdir runs/ --outdir figs/
# Requires: pip install tbparse pandas numpy matplotlib

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tbparse import SummaryReader


def ema(series, span=100):
    """Exponentially weighted moving average (like TB 'Smoothed')."""
    return series.ewm(span=span, adjust=False).mean()


def load_scalars(logdir):
    """Load scalars from TensorBoard logs, adding run information manually."""
    logdir = Path(logdir)
    
    # Find all subdirectories that contain TensorBoard event files
    run_dirs = []
    for subdir in logdir.iterdir():
        if subdir.is_dir():
            # Check if this directory contains TensorBoard event files
            event_files = list(subdir.glob("events.out.tfevents.*"))
            if event_files:
                run_dirs.append(subdir)
    
    if not run_dirs:
        print(f"No TensorBoard event files found in {logdir}")
        return pd.DataFrame(columns=['step', 'tag', 'value', 'run', 'wall_time'])
    
    # Load data from each run directory separately
    all_dfs = []
    for run_dir in run_dirs:
        try:
            reader = SummaryReader(str(run_dir), pivot=False)
            df = reader.scalars.copy()
            if not df.empty:
                # Add run name (use directory name)
                df['run'] = run_dir.name
                all_dfs.append(df)
        except Exception as e:
            print(f"Warning: Could not load data from {run_dir}: {e}")
            continue
    
    if not all_dfs:
        print("No valid TensorBoard data found")
        return pd.DataFrame(columns=['step', 'tag', 'value', 'run', 'wall_time'])
    
    # Combine all runs
    combined_df = pd.concat(all_dfs, ignore_index=True)
    
    # Normalize columns for safety
    combined_df["run"] = combined_df["run"].astype(str)
    combined_df["tag"] = combined_df["tag"].astype(str)
    
    # Add wall_time if not present (use step as approximation)
    if 'wall_time' not in combined_df.columns:
        combined_df['wall_time'] = combined_df['step']
    
    return combined_df


def summarize_runs(df, loss_tag="train/loss", lr_tag="train/learning_rate", smooth_span=100):
    # Final values (last logged step per run)
    last_loss = (
        df[df["tag"] == loss_tag]
        .sort_values(["run", "step"])
        .groupby("run")
        .tail(1)[["run", "step", "value"]]
        .rename(columns={"step": "final_step", "value": "final_loss"})
    )

    # Best loss (minimum) and step of min
    # Keep per-run min row
    best_idx = (
        df[df["tag"] == loss_tag]
        .sort_values(["run", "step"])
        .groupby("run")["value"].idxmin()
    )
    best_loss_rows = df.loc[best_idx, ["run", "step", "value"]]
    best_loss_rows = best_loss_rows.rename(columns={"step": "best_step", "value": "best_loss"})

    # Final LR
    last_lr = (
        df[df["tag"] == lr_tag]
        .sort_values(["run", "step"])
        .groupby("run")
        .tail(1)[["run", "value"]]
        .rename(columns={"value": "final_lr"})
    )

    # Training wall time (hours)
    # Compute elapsed per run from first to last wall_time
    wall = (
        df[df["tag"] == loss_tag]
        .sort_values(["run", "step"])
        .groupby("run")["wall_time"]
        .agg(["min", "max"])
        .reset_index()
    )
    wall["train_hours"] = (wall["max"] - wall["min"]) / 3600.0
    wall = wall[["run", "train_hours"]]

    table = last_loss.merge(best_loss_rows, on="run").merge(last_lr, on="run", how="left").merge(wall, on="run", how="left")

    # Optional smoothing value at final step (approximate TB 'Smoothed')
    # Build smoothed value by run, taking the value at the final step index
    smoothed_vals = []
    for r, g in df[df["tag"] == loss_tag].sort_values(["run", "step"]).groupby("run"):
        sm = ema(g["value"], span=smooth_span)
        smoothed_vals.append({"run": r, "final_loss_smoothed": sm.iloc[-1]})
    table = table.merge(pd.DataFrame(smoothed_vals), on="run", how="left")

    # Sort by final_step (helps readability)
    table = table.sort_values("final_step").reset_index(drop=True)

    # Nice rounding for LaTeX
    for col in ["final_loss", "best_loss", "final_lr", "train_hours", "final_loss_smoothed"]:
        if col in table.columns:
            table[col] = table[col].astype(float).round(6)

    return table


def plot_curves(df, tag, outpath, ylabel, title=None, smooth_span=100, legend_outside=True, show_raw_shadow=False, log_scale=False):
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

    # Make it wider for better visibility, especially for loss plots
    fig, ax = plt.subplots(figsize=(10, 4))
    
    # Define a color cycle for consistent colors between raw and smoothed
    import matplotlib.cm as cm
    colors = cm.get_cmap('tab10')(np.linspace(0, 1, 10))
    
    for i, (run, g) in enumerate(df[df["tag"] == tag].sort_values(["run", "step"]).groupby("run")):
        g = g[["step", "value"]].dropna()
        color = colors[i % len(colors)]
        
        if show_raw_shadow:
            # Plot raw signal as a light shadow behind the smoothed line
            ax.plot(g["step"], g["value"], 
                   color=color, alpha=0.2, linewidth=0.3, zorder=1)
        
        # Plot smoothed signal with thinner lines
        smoothed = ema(g["value"], smooth_span)
        ax.plot(g["step"], smoothed, 
               color=color, linewidth=1.0, label=run, zorder=2)

    # Set logarithmic scale if requested
    if log_scale:
        ax.set_yscale('log')

    ax.set_xlabel("Step")
    ax.set_ylabel(ylabel)
    if title:
        ax.set_title(title)

    # Add grid for better readability
    ax.grid(True, alpha=0.3)

    if legend_outside:
        ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=False, title="Run")
        fig.tight_layout(rect=(0, 0, 0.82, 1))
    else:
        ax.legend(frameon=False)
        fig.tight_layout()

    outpath.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(outpath, bbox_inches="tight", dpi=300)
    plt.close(fig)


def plot_combined_curves(df, loss_tag, lr_tag, outpath_png, table, smooth_span=100, log_scale=False):
    """Create a combined plot with loss, learning rate subplots, and summary table."""
    plt.rcParams.update({
        "pdf.fonttype": 42,      # editable text in Illustrator
        "ps.fonttype": 42,
        "font.size": 9,
        "axes.labelsize": 10,
        "axes.titlesize": 11,
        "legend.fontsize": 8,
        "xtick.labelsize": 8,
        "ytick.labelsize": 8,
    })

    # Create figure with 3 subplots: 2 plots on top, table on bottom
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(2, 2, height_ratios=[2, 1], hspace=0.3, wspace=0.3)
    
    ax1 = fig.add_subplot(gs[0, 0])  # Loss plot
    ax2 = fig.add_subplot(gs[0, 1])  # Learning rate plot
    ax3 = fig.add_subplot(gs[1, :])  # Table (spans both columns)
    
    # Define a color cycle for consistent colors between plots
    import matplotlib.cm as cm
    colors = cm.get_cmap('tab10')(np.linspace(0, 1, 10))
    
    # Plot 1: Loss curves
    for i, (run, g) in enumerate(df[df["tag"] == loss_tag].sort_values(["run", "step"]).groupby("run")):
        g = g[["step", "value"]].dropna()
        color = colors[i % len(colors)]
        
        # Plot raw signal as a light shadow
        ax1.plot(g["step"], g["value"], 
               color=color, alpha=0.2, linewidth=0.3, zorder=1)
        
        # Plot smoothed signal
        smoothed = ema(g["value"], smooth_span)
        ax1.plot(g["step"], smoothed, 
               color=color, linewidth=1.0, label=run, zorder=2)
    
    # Set logarithmic scale for loss if requested
    if log_scale:
        ax1.set_yscale('log')
    
    ax1.set_xlabel("Step")
    ax1.set_ylabel("Loss")
    ax1.set_title("Training Loss (EMA-smoothed)")
    ax1.grid(True, alpha=0.3)
    ax1.legend(frameon=False, loc='upper right')
    
    # Plot 2: Learning rate curves
    for i, (run, g) in enumerate(df[df["tag"] == lr_tag].sort_values(["run", "step"]).groupby("run")):
        g = g[["step", "value"]].dropna()
        color = colors[i % len(colors)]
        
        # No smoothing for learning rate (smooth_span=1)
        ax2.plot(g["step"], g["value"], 
               color=color, linewidth=1.0, label=run, zorder=2)
    
    ax2.set_xlabel("Step")
    ax2.set_ylabel("Learning Rate")
    ax2.set_title("Learning Rate Schedule")
    ax2.grid(True, alpha=0.3)
    ax2.legend(frameon=False, loc='upper right')
    
    # Plot 3: Summary table
    ax3.axis('tight')
    ax3.axis('off')
    
    # Prepare table data for display
    display_cols = ["run", "final_step", "final_loss", "final_loss_smoothed", "best_loss", "best_step", "final_lr", "train_hours"]
    display_cols = [c for c in display_cols if c in table.columns]
    
    # Create a cleaner table for display
    table_display = table[display_cols].copy()
    
    # Shorten run names for better display
    table_display["run"] = table_display["run"].str.replace("_RealFatMachine", "", regex=False)
    
    # Round numeric columns for better display
    numeric_cols = table_display.select_dtypes(include=[np.number]).columns
    for col in numeric_cols:
        if col in ['final_loss', 'final_loss_smoothed', 'best_loss']:
            table_display[col] = table_display[col].map(lambda x: f"{x:.4f}" if pd.notna(x) else "--")
        elif col in ['final_lr']:
            table_display[col] = table_display[col].map(lambda x: f"{x:.2e}" if pd.notna(x) else "--")
        elif col in ['train_hours']:
            table_display[col] = table_display[col].map(lambda x: f"{x:.1f}" if pd.notna(x) else "--")
        else:
            table_display[col] = table_display[col].map(lambda x: f"{x:,.0f}" if pd.notna(x) else "--")
    
    # Create the table
    table_plot = ax3.table(cellText=table_display.values,
                          colLabels=table_display.columns,
                          cellLoc='center',
                          loc='center')
    
    # Style the table
    table_plot.auto_set_font_size(False)
    table_plot.set_fontsize(8)
    table_plot.scale(1, 1.5)
    
    # Color the header
    for i in range(len(table_display.columns)):
        table_plot[(0, i)].set_facecolor('#E6E6E6')
        table_plot[(0, i)].set_text_props(weight='bold')
    
    ax3.set_title("Training Summary", pad=20)
    
    # Adjust layout and save
    outpath_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(outpath_png, bbox_inches="tight", dpi=300)
    plt.close(fig)


def table_to_latex(table, outpath_tex, caption="Training summary", label="tab:training_summary"):
    # Use booktabs for nicer tables
    cols = ["run", "final_step", "final_loss_smoothed", "final_loss", "best_loss", "best_step", "final_lr", "train_hours"]
    cols = [c for c in cols if c in table.columns]

    # Shorten run names a bit for print (optional)
    pretty = table.copy()
    pretty["run"] = pretty["run"].str.replace("_RealTallMachine", "", regex=False)

    latex = pretty[cols].to_latex(
        index=False,
        escape=True,
        longtable=False,
        caption=caption,
        label=label,
        float_format="%.6f",
        na_rep="--",
        column_format="lrrrrrrr"[: len(cols)]  # simple alignment
    )

    outpath_tex.parent.mkdir(parents=True, exist_ok=True)
    outpath_tex.write_text("% Auto-generated by make_figs.py\n\\begin{table}[t]\n\\centering\n" +
                           latex.split("\n", 1)[1].rsplit("\n", 1)[0] +  # strip standalone table env
                           "\n\\end{table}\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--logdir", required=True, help="Path to TensorBoard runs/ directory")
    ap.add_argument("--outdir", required=True, help="Where to write PDFs/SVGs/TEX")
    ap.add_argument("--loss_tag", default="train/loss")
    ap.add_argument("--lr_tag", default="train/learning_rate")
    ap.add_argument("--smooth_span", type=int, default=50, help="EMA smoothing span ~ TB 'Smoothed'")
    ap.add_argument("--log_scale", action="store_true", help="Use logarithmic scale for loss plot")
    args = ap.parse_args()

    logdir = Path(args.logdir)
    outdir = Path(args.outdir)

    df = load_scalars(logdir)
    table = summarize_runs(df, loss_tag=args.loss_tag, lr_tag=args.lr_tag, smooth_span=args.smooth_span)

    # Save LaTeX table
    table_to_latex(table, outdir / "metrics_table.tex",
                   caption="Run-level summary: final step, smoothed final loss, best loss, step at best, final LR, and wall-clock hours.",
                   label="tab:gr00t_summary")

    # Save individual figures
    plot_curves(df, args.loss_tag, outdir / "loss_plot.png", ylabel="Loss",
                title="Training loss (EMA-smoothed)", smooth_span=args.smooth_span, 
                show_raw_shadow=True, log_scale=args.log_scale)  # Enhanced visualization for loss
    plot_curves(df, args.lr_tag, outdir / "lr_plot.png", ylabel="Learning rate",
                title="Learning rate schedule", smooth_span=1)  # no smoothing for LR
    
    # Save combined plot
    plot_combined_curves(df, args.loss_tag, args.lr_tag, outdir / "combined_plot.png", table,
                        smooth_span=args.smooth_span, log_scale=args.log_scale)

    # Also export the raw table as CSV (handy for revision)
    table.to_csv(outdir / "metrics_table.csv", index=False)
    print(f"Done. Wrote: {outdir}")


if __name__ == "__main__":
    main()