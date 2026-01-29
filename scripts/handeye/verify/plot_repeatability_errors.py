#!/usr/bin/env python3
# ========================================================================
# DUPLICATE FILE - DO NOT USE
# ========================================================================
# This file is a duplicate of:
#   src/handeye_verify/handeye_verify/plot_repeatability_errors.py
#
# Please use the version in the handeye_verify package instead.
# This file is kept for backward compatibility but should be removed
# in a future cleanup.
# ========================================================================

"""
[DEPRECATED - Use src/handeye_verify/handeye_verify/plot_repeatability_errors.py]

Visualization tool for hand-eye calibration repeatability analysis.

Generates comprehensive plots:
1. 3D scatter plot of sample positions
2. XYZ error distribution over time
3. Error histogram with statistics
4. 2D top-view (XY plane) scatter
5. Radial error analysis
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import argparse
from typing import List, Tuple


def load_samples_from_csv(csv_file: str) -> Tuple[np.ndarray, List[float]]:
    """
    Load sample positions from CSV file.
    Returns: (samples_mm, stamp_spreads_ms)
    """
    import csv
    samples = []
    spreads = []

    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            samples.append([
                float(row['x_mm']),
                float(row['y_mm']),
                float(row['z_mm'])
            ])
            spreads.append(float(row['stamp_spread_ms']))

    return np.array(samples), spreads


def generate_sample_data(
    num_samples: int = 50,
    reference: np.ndarray = np.array([0.0, -650.0, 83.5]),
    noise_std: np.ndarray = np.array([3.0, 4.0, 15.0]),
    bias: np.ndarray = np.array([-1.5, -5.0, 0.0]),
) -> np.ndarray:
    """Generate simulated sample data for testing."""
    np.random.seed(42)
    samples = np.random.randn(num_samples, 3) * noise_std + reference + bias
    return samples


def compute_statistics(samples_mm: np.ndarray, reference_mm: np.ndarray) -> dict:
    """Compute comprehensive error statistics."""
    mean_mm = samples_mm.mean(axis=0)
    std_mm = samples_mm.std(axis=0, ddof=1)
    bias_mm = mean_mm - reference_mm

    # Compute per-sample errors
    errors_mm = samples_mm - reference_mm
    radial_errors_2d = np.linalg.norm(errors_mm[:, :2], axis=1)
    radial_errors_3d = np.linalg.norm(errors_mm, axis=1)

    return {
        "num_samples": len(samples_mm),
        "reference_mm": reference_mm,
        "mean_mm": mean_mm,
        "std_mm": std_mm,
        "bias_mm": bias_mm,
        "errors_mm": errors_mm,
        "radial_2d": radial_errors_2d,
        "radial_3d": radial_errors_3d,
        "max_error_3d": radial_errors_3d.max(),
        "rms_error_3d": np.sqrt(np.mean(radial_errors_3d**2)),
    }


def create_visualization(samples_mm: np.ndarray, reference_mm: np.ndarray, save_path: str = None):
    """Create comprehensive multi-panel visualization."""
    stats = compute_statistics(samples_mm, reference_mm)

    # Create figure with custom layout
    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(3, 3, figure=fig, hspace=0.3, wspace=0.3)

    # ========================================
    # Panel 1: 3D Scatter Plot (Top Left, Large)
    # ========================================
    ax1 = fig.add_subplot(gs[0:2, 0], projection='3d')

    # Plot samples
    ax1.scatter(
        samples_mm[:, 0],
        samples_mm[:, 1],
        samples_mm[:, 2],
        c=np.arange(len(samples_mm)),
        cmap='viridis',
        s=50,
        alpha=0.6,
        label='Samples'
    )

    # Plot reference point
    ax1.scatter(
        reference_mm[0],
        reference_mm[1],
        reference_mm[2],
        c='red',
        s=200,
        marker='*',
        label='Reference',
        edgecolors='black',
        linewidths=2
    )

    # Plot mean
    ax1.scatter(
        stats["mean_mm"][0],
        stats["mean_mm"][1],
        stats["mean_mm"][2],
        c='orange',
        s=150,
        marker='D',
        label='Mean',
        edgecolors='black',
        linewidths=1.5
    )

    ax1.set_xlabel('X (mm)', fontsize=10)
    ax1.set_ylabel('Y (mm)', fontsize=10)
    ax1.set_zlabel('Z (mm)', fontsize=10)
    ax1.set_title('3D Position Distribution', fontsize=12, fontweight='bold')
    ax1.legend(fontsize=9)
    ax1.grid(True, alpha=0.3)

    # ========================================
    # Panel 2: XYZ Errors Over Time (Top Right)
    # ========================================
    ax2 = fig.add_subplot(gs[0, 1:])

    time_idx = np.arange(len(samples_mm))

    ax2.plot(time_idx, stats["errors_mm"][:, 0], 'o-', label='X error', alpha=0.7, markersize=4)
    ax2.plot(time_idx, stats["errors_mm"][:, 1], 's-', label='Y error', alpha=0.7, markersize=4)
    ax2.plot(time_idx, stats["errors_mm"][:, 2], '^-', label='Z error', alpha=0.7, markersize=4)

    ax2.axhline(0, color='black', linestyle='--', linewidth=1, alpha=0.5)
    ax2.set_xlabel('Sample Index', fontsize=10)
    ax2.set_ylabel('Error (mm)', fontsize=10)
    ax2.set_title('Error vs Time', fontsize=12, fontweight='bold')
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.3)

    # ========================================
    # Panel 3: Error Histograms (Middle Right)
    # ========================================
    ax3 = fig.add_subplot(gs[1, 1:])

    bins = 20
    ax3.hist(stats["errors_mm"][:, 0], bins=bins, alpha=0.5, label='X', color='C0')
    ax3.hist(stats["errors_mm"][:, 1], bins=bins, alpha=0.5, label='Y', color='C1')
    ax3.hist(stats["errors_mm"][:, 2], bins=bins, alpha=0.5, label='Z', color='C2')

    ax3.axvline(0, color='black', linestyle='--', linewidth=1, alpha=0.5)
    ax3.set_xlabel('Error (mm)', fontsize=10)
    ax3.set_ylabel('Frequency', fontsize=10)
    ax3.set_title('Error Distribution', fontsize=12, fontweight='bold')
    ax3.legend(fontsize=9)
    ax3.grid(True, alpha=0.3, axis='y')

    # ========================================
    # Panel 4: XY Top View (Bottom Left)
    # ========================================
    ax4 = fig.add_subplot(gs[2, 0])

    ax4.scatter(
        samples_mm[:, 0],
        samples_mm[:, 1],
        c=np.arange(len(samples_mm)),
        cmap='viridis',
        s=50,
        alpha=0.6
    )

    ax4.scatter(
        reference_mm[0],
        reference_mm[1],
        c='red',
        s=200,
        marker='*',
        edgecolors='black',
        linewidths=2,
        label='Reference'
    )

    # Draw std deviation ellipse
    from matplotlib.patches import Ellipse
    ellipse = Ellipse(
        xy=(stats["mean_mm"][0], stats["mean_mm"][1]),
        width=stats["std_mm"][0] * 4,  # 2-sigma
        height=stats["std_mm"][1] * 4,
        angle=0,
        edgecolor='orange',
        facecolor='none',
        linewidth=2,
        linestyle='--',
        label='2σ ellipse'
    )
    ax4.add_patch(ellipse)

    ax4.set_xlabel('X (mm)', fontsize=10)
    ax4.set_ylabel('Y (mm)', fontsize=10)
    ax4.set_title('XY Plane (Top View)', fontsize=12, fontweight='bold')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.axis('equal')

    # ========================================
    # Panel 5: Radial Error (Bottom Center)
    # ========================================
    ax5 = fig.add_subplot(gs[2, 1])

    ax5.plot(time_idx, stats["radial_2d"], 'o-', label='2D (XY)', alpha=0.7, markersize=4)
    ax5.plot(time_idx, stats["radial_3d"], 's-', label='3D (XYZ)', alpha=0.7, markersize=4)

    ax5.axhline(stats["radial_2d"].mean(), color='C0', linestyle='--', linewidth=1, alpha=0.7)
    ax5.axhline(stats["radial_3d"].mean(), color='C1', linestyle='--', linewidth=1, alpha=0.7)

    ax5.set_xlabel('Sample Index', fontsize=10)
    ax5.set_ylabel('Radial Error (mm)', fontsize=10)
    ax5.set_title('Radial Error', fontsize=12, fontweight='bold')
    ax5.legend(fontsize=9)
    ax5.grid(True, alpha=0.3)

    # ========================================
    # Panel 6: Statistics Table (Bottom Right)
    # ========================================
    ax6 = fig.add_subplot(gs[2, 2])
    ax6.axis('off')

    # Create statistics text
    stats_text = f"""
STATISTICS SUMMARY

Samples: {stats["num_samples"]}

Reference (mm):
  X: {stats["reference_mm"][0]:+.2f}
  Y: {stats["reference_mm"][1]:+.2f}
  Z: {stats["reference_mm"][2]:+.2f}

Bias (mm):
  X: {stats["bias_mm"][0]:+.3f}
  Y: {stats["bias_mm"][1]:+.3f}
  Z: {stats["bias_mm"][2]:+.3f}

Std Dev (mm):
  X: {stats["std_mm"][0]:.3f}
  Y: {stats["std_mm"][1]:.3f}
  Z: {stats["std_mm"][2]:.3f}

Radial Error:
  2D Mean: {stats["radial_2d"].mean():.3f} mm
  3D Mean: {stats["radial_3d"].mean():.3f} mm
  3D RMS:  {stats["rms_error_3d"]:.3f} mm
  3D Max:  {stats["max_error_3d"]:.3f} mm

Norm Bias: {np.linalg.norm(stats["bias_mm"]):.3f} mm
"""

    ax6.text(
        0.05, 0.95,
        stats_text,
        transform=ax6.transAxes,
        fontsize=9,
        verticalalignment='top',
        fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3)
    )

    # Main title
    fig.suptitle(
        'Hand-Eye Calibration Repeatability Analysis',
        fontsize=16,
        fontweight='bold',
        y=0.98
    )

    # Save or show
    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {save_path}")
    else:
        plt.show()

    plt.close()

    return stats


def create_simple_plot(samples_mm: np.ndarray, reference_mm: np.ndarray, save_path: str = None):
    """Create a simpler 2-panel plot for quick analysis."""
    stats = compute_statistics(samples_mm, reference_mm)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Panel 1: XYZ errors over time
    ax1 = axes[0]
    time_idx = np.arange(len(samples_mm))
    ax1.plot(time_idx, stats["errors_mm"][:, 0], 'o-', label=f'X (bias={stats["bias_mm"][0]:+.2f}mm)', alpha=0.7)
    ax1.plot(time_idx, stats["errors_mm"][:, 1], 's-', label=f'Y (bias={stats["bias_mm"][1]:+.2f}mm)', alpha=0.7)
    ax1.plot(time_idx, stats["errors_mm"][:, 2], '^-', label=f'Z (bias={stats["bias_mm"][2]:+.2f}mm)', alpha=0.7)
    ax1.axhline(0, color='black', linestyle='--', linewidth=1)
    ax1.set_xlabel('Sample Index')
    ax1.set_ylabel('Error (mm)')
    ax1.set_title('Position Error vs Time')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Panel 2: Error histograms
    ax2 = axes[1]
    ax2.hist(stats["errors_mm"][:, 0], bins=15, alpha=0.5, label=f'X (σ={stats["std_mm"][0]:.2f}mm)')
    ax2.hist(stats["errors_mm"][:, 1], bins=15, alpha=0.5, label=f'Y (σ={stats["std_mm"][1]:.2f}mm)')
    ax2.hist(stats["errors_mm"][:, 2], bins=15, alpha=0.5, label=f'Z (σ={stats["std_mm"][2]:.2f}mm)')
    ax2.axvline(0, color='black', linestyle='--', linewidth=1)
    ax2.set_xlabel('Error (mm)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Error Distribution')
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis='y')

    plt.suptitle(
        f'Repeatability Analysis (N={stats["num_samples"]}, Norm Bias={np.linalg.norm(stats["bias_mm"]):.2f}mm)',
        fontsize=13,
        fontweight='bold'
    )
    plt.tight_layout()

    if save_path:
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {save_path}")
    else:
        plt.show()

    plt.close()

    return stats


def main():
    parser = argparse.ArgumentParser(description='Visualize hand-eye calibration repeatability errors')
    parser.add_argument('--csv', type=str, help='Path to CSV file with sample data')
    parser.add_argument('--reference', type=float, nargs=3, default=[0.0, -650.0, 83.5],
                        help='Reference point in mm (X Y Z)')
    parser.add_argument('--output', type=str, default='repeatability_analysis.png',
                        help='Output plot filename')
    parser.add_argument('--simple', action='store_true',
                        help='Create simple 2-panel plot instead of comprehensive view')
    parser.add_argument('--samples', type=int, default=50,
                        help='Number of samples for simulated data (if --csv not provided)')

    args = parser.parse_args()

    reference_mm = np.array(args.reference)

    # Load data from CSV or generate simulated data
    if args.csv:
        print(f"Loading samples from: {args.csv}")
        samples_mm, _ = load_samples_from_csv(args.csv)
        print(f"Loaded {len(samples_mm)} samples")
    else:
        print("No CSV provided, using simulated data for demonstration...")
        samples_mm = generate_sample_data(
            num_samples=args.samples,
            reference=reference_mm,
            noise_std=np.array([3.0, 4.0, 15.0]),
            bias=np.array([-1.5, -5.0, 0.0])
        )

    if args.simple:
        stats = create_simple_plot(samples_mm, reference_mm, args.output)
    else:
        stats = create_visualization(samples_mm, reference_mm, args.output)

    print("\n" + "=" * 70)
    print("STATISTICS SUMMARY")
    print("=" * 70)
    print(f"Samples:        {stats['num_samples']}")
    print(f"Bias (mm):      X={stats['bias_mm'][0]:+.3f}, Y={stats['bias_mm'][1]:+.3f}, Z={stats['bias_mm'][2]:+.3f}")
    print(f"Std Dev (mm):   X={stats['std_mm'][0]:.3f}, Y={stats['std_mm'][1]:.3f}, Z={stats['std_mm'][2]:.3f}")
    print(f"Norm Bias:      {np.linalg.norm(stats['bias_mm']):.3f} mm")
    print(f"RMS Error (3D): {stats['rms_error_3d']:.3f} mm")
    print(f"Max Error (3D): {stats['max_error_3d']:.3f} mm")
    print("=" * 70)


if __name__ == "__main__":
    main()
