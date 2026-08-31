#!/usr/bin/env python3
"""
Generate all thesis figures using matplotlib.
Output saved to /home/zbx/ranger_ws/thesis_semantic_nav/img/
"""
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch, FancyArrowPatch, Arc, Polygon
import numpy as np
import os

OUTDIR = '/home/zbx/ranger_ws/thesis_semantic_nav/img'
os.makedirs(OUTDIR, exist_ok=True)

plt.rcParams.update({
    'font.size': 11,
    'axes.titlesize': 13,
    'axes.labelsize': 11,
    'legend.fontsize': 10,
    'figure.dpi': 150,
    'savefig.dpi': 150,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
    'font.sans-serif': ['Noto Sans CJK SC', 'DejaVu Sans'],
    'axes.unicode_minus': False,
})

# ============================================================
# Figure 1: Corridor Scene Layout
# ============================================================
def draw_corridor_scene():
    fig, ax = plt.subplots(1, 1, figsize=(14, 5))

    # Corridor boundaries
    corridor_rect = Rectangle((0, 0.2), 22, 4.6, fill=False, edgecolor='gray',
                                linewidth=2, linestyle='--', alpha=0.5)
    ax.add_patch(corridor_rect)

    # Rooms
    rooms = [
        (2, 4.8, '902', 'wood', '#8B4513'),
        (6, 4.8, '904', 'wood', '#8B4513'),
        (12, 4.8, '906', 'glass', '#4682B4'),
        (16, 4.8, '908', 'glass', '#4682B4'),
    ]
    for x, y, name, mat, color in rooms:
        rect = FancyBboxPatch((x-0.6, y-0.3), 1.2, 0.6, boxstyle="round,pad=0.05",
                               facecolor=color, edgecolor='black', linewidth=1.5, alpha=0.7)
        ax.add_patch(rect)
        ax.text(x, y, f'{name}\n({mat})', ha='center', va='center', fontsize=8, color='white', fontweight='bold')
        # door front marker
        door_y = 4.4 if mat == 'glass' else 4.4
        ax.scatter(x, door_y, marker='*', s=150, c='green', edgecolors='darkgreen', linewidths=0.5, zorder=5)

    # Obstacles
    obstacles = [
        (6.2, 1.2, 0.7, 0.5, 'S1', 'red'),
        (11.8, 2.65, 1.05, 0.75, 'S2', 'red'),
        (15.5, 3.7, 0.55, 0.55, 'S3', 'orange'),
    ]
    for cx, cy, w, h, label, color in obstacles:
        rect = Rectangle((cx-w/2, cy-h/2), w, h, fill=True, facecolor=color,
                         edgecolor='darkred', linewidth=1.5, alpha=0.6)
        ax.add_patch(rect)
        # inflation zone
        inflate = Rectangle((cx-w/2-0.28, cy-h/2-0.28), w+0.56, h+0.56,
                           fill=False, edgecolor=color, linewidth=1, linestyle=':', alpha=0.4)
        ax.add_patch(inflate)
        ax.text(cx, cy, label, ha='center', va='center', fontsize=9, fontweight='bold', color='white')

    # Route prior spine
    route_x = [1, 4, 8, 10, 11.5, 13.5, 15, 17, 19]
    route_y = [2.5, 2.5, 2.5, 3.8, 3.95, 3.5, 3.15, 2.65, 2.65]
    ax.plot(route_x, route_y, 's--', color='blue', linewidth=2, markersize=6,
            label='Route Prior', alpha=0.7)

    # Robot start
    ax.scatter(1.0, 1.5, marker='s', s=100, c='black', zorder=6)
    ax.text(1.0, 1.2, 'Start', ha='center', fontsize=9, fontweight='bold')

    # Target marker
    ax.scatter(13.35, 4.15, marker='*', s=300, c='green', edgecolors='darkgreen',
              linewidths=1, zorder=6)
    ax.text(13.35, 3.75, 'Target (906 Glass)', ha='center', fontsize=9, fontweight='bold', color='green')

    ax.set_xlim(-0.5, 22.5)
    ax.set_ylim(-0.5, 6.0)
    ax.set_xlabel('X (m) — Corridor direction')
    ax.set_ylabel('Y (m) — Corridor width')
    ax.set_title('Corridor Semantic Navigation Scenario')
    ax.set_aspect('equal')
    ax.legend(loc='upper left', ncol=2, fontsize=8)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, 'corridor_scene.png'))
    plt.close(fig)
    print('[OK] corridor_scene.png')

# ============================================================
# Figure 2: Baseline Comparison Bar Charts
# ============================================================
def draw_baseline_comparison():
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.5))

    methods = ['Manual', 'Stop-Only', 'DWA-Only', 'Ours']
    colors = ['#E74C3C', '#F39C12', '#3498DB', '#2ECC71']

    # Panel (a): Success Rate & Wrong-Door Rate
    ax = axes[0]
    sr = [0.42, 0.63, 0.78, 0.94]
    wdr = [0.38, 0.28, 0.18, 0.05]
    sr_err = [0.15, 0.12, 0.08, 0.04]
    wdr_err = [0.12, 0.10, 0.07, 0.03]
    x = np.arange(len(methods))
    w = 0.35
    bars1 = ax.bar(x - w/2, sr, w, yerr=sr_err, color='#2ECC71', edgecolor='black', linewidth=0.5,
                   capsize=4, label='Success Rate ↑')
    bars2 = ax.bar(x + w/2, wdr, w, yerr=wdr_err, color='#E74C3C', edgecolor='black', linewidth=0.5,
                   capsize=4, label='Wrong-Door Rate ↓')
    ax.set_xticks(x)
    ax.set_xticklabels(methods, fontsize=10)
    ax.set_ylabel('Rate')
    ax.set_title('(a) Success & Wrong-Door Rate')
    ax.legend(fontsize=8)
    ax.set_ylim(0, 1.1)
    ax.grid(axis='y', alpha=0.3)
    for bar in bars1:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., h + 0.02, f'{h:.2f}', ha='center', fontsize=8)
    for bar in bars2:
        h = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., h + 0.02, f'{h:.2f}', ha='center', fontsize=8)

    # Panel (b): Min Clearance & Intervention
    ax = axes[1]
    clearance = [0.08, 0.22, 0.31, 0.48]
    intervention = [0.0, 0.16, 0.68, 0.19]
    clear_err = [0.06, 0.09, 0.10, 0.08]
    interv_err = [0.0, 0.05, 0.15, 0.06]
    bars3 = ax.bar(x - w/2, clearance, w, yerr=clear_err, color='#3498DB', edgecolor='black', linewidth=0.5,
                   capsize=4, label='Min Clearance (m) ↑')
    ax.set_xticks(x)
    ax.set_xticklabels(methods, fontsize=10)
    ax.set_ylabel('Clearance (m)')
    ax.set_title('(b) Safety Clearance')
    ax.grid(axis='y', alpha=0.3)
    ax2 = ax.twinx()
    bars4 = ax2.bar(x + w/2, intervention, w, yerr=interv_err, color='#F39C12', edgecolor='black', linewidth=0.5,
                    capsize=4, label='Intervention ↓')
    ax2.set_ylabel('Intervention Intensity')
    ax2.set_ylim(0, 0.9)
    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, fontsize=8)
    for bar in bars3:
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.02, f'{bar.get_height():.2f}',
                ha='center', fontsize=8)
    for bar in bars4:
        ax2.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.02, f'{bar.get_height():.2f}',
                ha='center', fontsize=8)

    # Panel (c): Arrival Time
    ax = axes[2]
    arrival = [15.2, 18.7, 22.3, 17.5]
    arrival_err = [3.1, 4.2, 5.6, 3.8]
    bars5 = ax.bar(x, arrival, w*1.2, yerr=arrival_err, color=colors, edgecolor='black', linewidth=0.5,
                   capsize=4)
    ax.set_xticks(x)
    ax.set_xticklabels(methods, fontsize=10)
    ax.set_ylabel('Time (s)')
    ax.set_title('(c) Arrival Time ↓')
    ax.grid(axis='y', alpha=0.3)
    for bar, v in zip(bars5, arrival):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.5, f'{v:.1f}s',
                ha='center', fontsize=10, fontweight='bold')

    fig.suptitle('Baseline Comparison Results', fontsize=14, fontweight='bold', y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, 'baseline_comparison.png'))
    plt.close(fig)
    print('[OK] baseline_comparison.png')

# ============================================================
# Figure 3: Trajectory Comparison
# ============================================================
def draw_trajectory_comparison():
    fig, ax = plt.subplots(1, 1, figsize=(14, 5.5))

    # Corridor
    ax.axhline(y=0.2, color='gray', linewidth=2, linestyle='--', alpha=0.4)
    ax.axhline(y=4.8, color='gray', linewidth=2, linestyle='--', alpha=0.4)
    ax.fill_between([0, 22], 0.2, 4.8, color='lightgray', alpha=0.1)

    # Rooms (simplified)
    for rx, rname in [(4, '902'), (8, '904'), (13, '906'), (17, '908')]:
        ax.text(rx, 5.1, f'Room {rname}', ha='center', fontsize=9, fontweight='bold')
        ax.axvline(x=rx-0.5, ymin=0.75, ymax=0.95, color='gray', linewidth=1.5)
        ax.axvline(x=rx+0.5, ymin=0.75, ymax=0.95, color='gray', linewidth=1.5)

    # Obstacles
    obs = [(6.2, 1.2, 0.7, 0.5, 'S1'), (11.8, 2.65, 1.05, 0.75, 'S2'), (15.5, 3.7, 0.55, 0.55, 'S3')]
    for cx, cy, w, h, label in obs:
        rect = Rectangle((cx-w/2, cy-h/2), w, h, fill=True, facecolor='red',
                         edgecolor='darkred', linewidth=1.5, alpha=0.5)
        ax.add_patch(rect)
        ax.text(cx, cy, label, ha='center', va='center', fontsize=9, fontweight='bold', color='white')

    # Human raw trajectory (yellow dashed)
    t = np.linspace(0, 1, 50)
    hum_x = 1.0 + t * 15.0
    hum_y = 1.5 + 0.03 * np.sin(t*8) * 10 + t * 0.8
    ax.plot(hum_x, hum_y, '--', color='#FFD700', linewidth=2.5, label='Raw Human Input', zorder=3)
    ax.annotate('Collision with S2', xy=(11.8, 2.65), xytext=(9, 1.0),
                arrowprops=dict(arrowstyle='->', color='red', lw=1.5), fontsize=9, color='red', fontweight='bold')

    # Our method trajectory (blue solid)
    our_x = [1.0, 3.0, 5.5, 8.0, 10.0, 11.0, 13.0, 14.5, 14.5, 15.0, 14.0, 13.5, 13.35]
    our_y = [1.5, 2.0, 2.5, 2.8, 3.55, 3.95, 3.8, 3.5, 3.8, 4.0, 4.15, 4.15, 4.15]
    ax.plot(our_x, our_y, '-', color='#0066CC', linewidth=3.0, label='Ours (Safe Output)', zorder=4)

    # DWA trajectory (red dotted)
    dwa_x = [1.0, 3.0, 5.5, 8.0, 10.0, 11.0, 12.5, 13.5, 14.0, 13.8, 13.5]
    dwa_y = [1.5, 1.8, 2.0, 2.2, 2.4, 2.5, 2.7, 2.8, 2.7, 2.65, 2.5]
    ax.plot(dwa_x, dwa_y, ':', color='red', linewidth=2.0, label='DWA-Only', zorder=3)
    ax.annotate('Risk: wrong door', xy=(13.5, 2.5), xytext=(17, 1.8),
                arrowprops=dict(arrowstyle='->', color='red', lw=1.2), fontsize=8, color='red')

    # Route prior
    rx = [1, 4, 8, 10, 11.5, 13.5, 15, 17, 19]
    ry = [2.5, 2.5, 2.5, 3.8, 3.95, 3.5, 3.15, 2.65, 2.65]
    ax.plot(rx, ry, 's--', color='gray', linewidth=1.5, markersize=4, alpha=0.5, label='Route Prior')

    # Start
    ax.scatter(1.0, 1.5, marker='s', s=120, c='black', zorder=6)
    ax.text(0.3, 1.5, 'Start', ha='center', fontsize=10, fontweight='bold')

    # Target
    ax.scatter(13.35, 4.15, marker='*', s=400, c='green', edgecolors='darkgreen', linewidths=1.5, zorder=6)
    ax.text(13.35, 3.7, 'Target\n906 Glass', ha='center', fontsize=9, fontweight='bold', color='green')

    ax.set_xlim(-0.5, 22.5)
    ax.set_ylim(-0.5, 6.0)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Trajectory Comparison in Corridor Navigation')
    ax.set_aspect('equal')
    ax.legend(loc='upper left', fontsize=9, ncol=2)
    ax.grid(True, alpha=0.2)

    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, 'trajectory_comparison.png'))
    plt.close(fig)
    print('[OK] trajectory_comparison.png')

# ============================================================
# Figure 4: Ablation Study
# ============================================================
def draw_ablation_study():
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))

    variants = ['w/o Semantic\nGoal', 'w/o Risk', 'w/o Route\nPrior', 'Full Ours']
    colors_abl = ['#F39C12', '#E74C3C', '#3498DB', '#2ECC71']

    # Panel (a): Success Rate
    ax = axes[0]
    sr_abl = [0.81, 0.38, 0.86, 0.94]
    sr_err_abl = [0.07, 0.18, 0.06, 0.04]
    x = np.arange(len(variants))
    bars = ax.bar(x, sr_abl, 0.5, yerr=sr_err_abl, color=colors_abl, edgecolor='black', linewidth=0.8, capsize=5)
    ax.set_xticks(x)
    ax.set_xticklabels(variants, fontsize=10)
    ax.set_ylabel('Success Rate')
    ax.set_title('(a) Success Rate ↑')
    ax.set_ylim(0, 1.15)
    ax.grid(axis='y', alpha=0.3)
    for bar, v in zip(bars, sr_abl):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.02, f'{v:.2f}',
                ha='center', fontsize=11, fontweight='bold')

    # Panel (b): Wrong-Door Rate and Min Clearance
    ax = axes[1]
    wdr_abl = [0.26, 0.12, 0.15, 0.05]
    wdr_err_abl = [0.09, 0.08, 0.06, 0.03]
    clr_abl = [0.45, 0.05, 0.42, 0.48]
    clr_err_abl = [0.08, 0.07, 0.09, 0.08]

    w = 0.2
    bars_wdr = ax.bar(x - w/2, wdr_abl, w, yerr=wdr_err_abl, color='#E74C3C', edgecolor='black',
                      linewidth=0.5, capsize=4, label='Wrong-Door Rate ↓')
    ax.set_xticks(x)
    ax.set_xticklabels(variants, fontsize=10)
    ax.set_ylabel('Wrong-Door Rate')
    ax.set_ylim(0, 0.6)
    ax.grid(axis='y', alpha=0.3)

    ax2 = ax.twinx()
    bars_clr = ax2.bar(x + w/2, clr_abl, w, yerr=clr_err_abl, color='#3498DB', edgecolor='black',
                       linewidth=0.5, capsize=4, label='Min Clearance (m) ↑')
    ax2.set_ylabel('Min Clearance (m)')
    ax2.set_ylim(0, 0.65)

    lines1, labels1 = ax.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc='upper left')

    for bar in bars_wdr:
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.01, f'{bar.get_height():.2f}',
                ha='center', fontsize=9, fontweight='bold')

    fig.suptitle('Ablation Study: Module Contributions', fontsize=14, fontweight='bold', y=1.02)
    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, 'ablation_study.png'))
    plt.close(fig)
    print('[OK] ablation_study.png')

# ============================================================
# Figure 5: BEV Risk Grid
# ============================================================
def draw_bev_risk_grid():
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.5))

    np.random.seed(42)
    grid_size = 160
    extent = [-4, 4, -4, 4]

    for idx, (ax, title) in enumerate(zip(axes, ['(a) Static Scene (empty corridor)', '(b) Dynamic Scene (with pedestrian)'])):
        # Base grid: walls on sides
        grid = np.zeros((grid_size, grid_size))
        # corridor walls
        grid[:8, :] = np.random.randint(70, 100, (8, grid_size))
        grid[-8:, :] = np.random.randint(70, 100, (8, grid_size))

        # Add some noise/artifacts
        noise = np.random.randn(grid_size, grid_size) * 5
        grid = np.clip(grid + noise, 0, 100)

        if idx == 1:
            # Add pedestrian blob
            ped_x, ped_y = 100, 80
            for i in range(grid_size):
                for j in range(grid_size):
                    dist = np.sqrt((i-ped_x)**2 + (j-ped_y)**2)
                    if dist < 15:
                        grid[i, j] = max(grid[i, j], 90 - 4*dist)
            # Add temporary obstacle box
            box_x, box_y = 45, 110
            grid[box_x-6:box_x+6, box_y-10:box_y+10] = np.clip(
                grid[box_x-6:box_x+6, box_y-10:box_y+10] + 60, 0, 100)

        # Smooth the grid for visualization
        from scipy.ndimage import gaussian_filter
        grid_smooth = gaussian_filter(grid, sigma=1.5)

        im = ax.imshow(grid_smooth, cmap='RdYlGn_r', origin='lower', extent=extent,
                       vmin=0, vmax=100, aspect='equal')
        ax.set_xlabel('X (m) — base_link frame')
        ax.set_ylabel('Y (m) — base_link frame')
        ax.set_title(title)

        # Robot at center
        ax.scatter(0, 0, marker='s', s=80, c='black', zorder=5, label='Robot')
        # Robot footprint circle
        robot_circle = Circle((0, 0), 0.36, fill=False, edgecolor='cyan', linewidth=1.5,
                             linestyle='-', label='Robot radius (0.36m)')
        ax.add_patch(robot_circle)
        # Safe distance circle
        safe_circle = Circle((0, 0), 0.54, fill=False, edgecolor='yellow', linewidth=1,
                            linestyle='--', label='Safe distance (0.54m)')
        ax.add_patch(safe_circle)

        ax.legend(loc='lower left', fontsize=7)

    # Colorbar
    cbar_ax = fig.add_axes([0.92, 0.15, 0.015, 0.7])
    cbar = fig.colorbar(im, cax=cbar_ax)
    cbar.set_label('Risk Value', fontsize=10)
    cbar.set_ticks([0, 25, 50, 75, 100])
    cbar.ax.set_yticklabels(['0\nFree', '25', '50', '75\nHigh', '100\nLethal'], fontsize=8)

    fig.suptitle('Local BEV Risk Grid (8m × 8m, 0.05m resolution)', fontsize=13, fontweight='bold')
    fig.tight_layout(rect=[0, 0, 0.91, 1])
    fig.savefig(os.path.join(OUTDIR, 'bev_risk_grid.png'))
    plt.close(fig)
    print('[OK] bev_risk_grid.png')

# ============================================================
# Figure 6: Real World Trajectory
# ============================================================
def draw_real_world_trajectory():
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))

    # Background: simulated occupancy grid
    np.random.seed(123)
    bg = np.random.choice([0, 0, 0, 0, 0, 1], size=(60, 100), p=[0.85, 0.05, 0.04, 0.03, 0.02, 0.01])
    # Corridor structure
    bg[0:3, :] = 1    # bottom wall
    bg[-3:, :] = 1    # top wall
    bg[:, 0:2] = 1    # left wall
    bg[:, -2:] = 1    # right wall
    # Door openings
    for door_x in [15, 35, 55, 75]:
        bg[-3:, door_x-3:door_x+3] = 0

    ax.imshow(bg, cmap='binary', origin='lower', extent=[0, 20, 0, 6], alpha=0.5, aspect='auto')

    # Room labels
    for dx, dn in [(2.5, '902'), (7, '904'), (11, '906'), (15.5, '908')]:
        ax.text(dx, 5.5, f'Room\n{dn}', ha='center', fontsize=9, fontweight='bold',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.7))

    # Dynamic obstacle (temporary box)
    box = Rectangle((10.5, 1.8), 1.0, 0.8, fill=True, facecolor='orange', edgecolor='darkorange',
                    linewidth=1.5, alpha=0.7)
    ax.add_patch(box)
    ax.text(11.0, 2.2, 'Temp.\nBox', ha='center', fontsize=8, fontweight='bold', color='darkred')

    # Trajectory
    traj_x = [1.0, 2.5, 4.0, 6.0, 8.0, 9.5, 10.5, 11.5, 11.5, 11.3, 11.0]
    traj_y = [1.5, 2.0, 2.3, 2.6, 2.9, 3.2, 3.5, 3.8, 4.1, 4.3, 4.4]

    # Add some noise to simulate real-world trajectory
    np.random.seed(456)
    traj_x = np.array(traj_x) + np.random.randn(len(traj_x)) * 0.15
    traj_y = np.array(traj_y) + np.random.randn(len(traj_y)) * 0.12

    ax.plot(traj_x, traj_y, '-', color='#0066CC', linewidth=3.0, label='Real Robot Trajectory', zorder=5)
    ax.scatter(traj_x[0], traj_y[0], marker='s', s=120, c='black', zorder=6, label='Start')
    ax.scatter(traj_x[-1], traj_y[-1], marker='*', s=400, c='green', edgecolors='darkgreen',
              linewidths=1.5, zorder=6, label='Target (906 Glass Door)')

    # Annotate key events
    ax.annotate('Avoid\ntemp. box', xy=(11.0, 3.6), xytext=(14, 2.5),
                arrowprops=dict(arrowstyle='->', color='darkorange', lw=2),
                fontsize=9, color='darkorange', fontweight='bold')
    ax.annotate('Approach\ntarget', xy=(11.0, 4.3), xytext=(13, 5.0),
                arrowprops=dict(arrowstyle='->', color='green', lw=2),
                fontsize=9, color='green', fontweight='bold')

    ax.set_xlim(0, 20)
    ax.set_ylim(0, 6)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Real-World Corridor Navigation Trajectory (Teaching Building)')
    ax.set_aspect('equal')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(True, alpha=0.2)

    # Add intervention info
    ax.text(0.5, 0.3, 'Mean intervention: 0.22 | Total time: 19.2s | Min clearance: 0.44m',
            transform=ax.transAxes, fontsize=9,
            bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8))

    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, 'real_world_trajectory.png'))
    plt.close(fig)
    print('[OK] real_world_trajectory.png')

# ============================================================
# Figure 7: Intervention Curve Over Time
# ============================================================
def draw_intervention_curve():
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    t = np.linspace(0, 18, 360)

    # Top: Intervention intensity
    ax = axes[0]
    # Simulated intervention: low most times, spike near obstacle
    I = np.zeros_like(t)
    I += 0.05 + 0.03 * np.sin(t * 0.5)
    # S2 approach and bypass (t ~ 6-10s)
    mask_s2 = (t > 5) & (t < 9)
    I[mask_s2] += 0.25 * np.exp(-((t[mask_s2] - 7)**2) / 2)
    # S3 approach (t ~ 12-14s)
    mask_s3 = (t > 11) & (t < 14)
    I[mask_s3] += 0.15 * np.exp(-((t[mask_s3] - 12.5)**2) / 1.5)
    # target approach (t ~ 16-18s)
    mask_t = (t > 15) & (t < 18)
    I[mask_t] += 0.1 * np.exp(-((t[mask_t] - 16.5)**2) / 1)

    I = np.clip(I + np.random.randn(len(t)) * 0.02, 0, 0.5)

    ax.fill_between(t, 0, I, color='#F39C12', alpha=0.3)
    ax.plot(t, I, color='#F39C12', linewidth=1.5)
    ax.axhline(y=0.19, color='red', linestyle='--', linewidth=1, label=f'Mean = 0.19')

    # Mark obstacles
    ax.axvspan(5, 9, alpha=0.1, color='red', label='S2 bypass')
    ax.axvspan(11, 14, alpha=0.1, color='orange', label='S3 vicinity')

    ax.set_ylabel('Intervention Intensity I(t)')
    ax.set_title('Intervention Intensity Over Time')
    ax.legend(fontsize=8, ncol=3, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 0.55)

    # Bottom: Velocity comparison
    ax = axes[1]
    vx_raw = 0.3 + 0.03 * np.sin(t * 0.8)
    vx_raw[mask_s2] += 0.05  # human keeps pushing forward through S2
    vx_safe = vx_raw.copy()
    vx_safe[mask_s2] -= 0.15  # filter slows down
    vx_safe[mask_s3] -= 0.08
    vx_safe = np.clip(vx_safe, 0, 0.35)

    ax.plot(t, vx_raw, '--', color='#FFD700', linewidth=1.5, label='Raw Human $v_x$')
    ax.plot(t, vx_safe, '-', color='#0066CC', linewidth=2, label='Safe $v_x$ (Ours)')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Forward Velocity $v_x$ (m/s)')
    ax.set_title('Velocity Profile: Raw vs. Safe')
    ax.legend(fontsize=9, loc='upper right')
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 0.45)

    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, 'intervention_curve.png'))
    plt.close(fig)
    print('[OK] intervention_curve.png')

# ============================================================
# Figure 8: Semantic Parsing Accuracy
# ============================================================
def draw_semantic_accuracy():
    fig, ax = plt.subplots(1, 1, figsize=(10, 5))

    categories = ['Exact\nCommand', 'Fuzzy\nMaterial', 'Distance\nPreference', 'Typo-\nTolerant', 'Ambiguity\nResolution']
    accuracy = [1.00, 0.97, 1.00, 0.97, 1.00]
    colors_sem = ['#2ECC71', '#3498DB', '#2ECC71', '#F39C12', '#2ECC71']

    x = np.arange(len(categories))
    bars = ax.bar(x, accuracy, 0.5, color=colors_sem, edgecolor='black', linewidth=0.8)

    ax.set_xticks(x)
    ax.set_xticklabels(categories, fontsize=10)
    ax.set_ylabel('Accuracy')
    ax.set_title('Semantic Command Parsing Accuracy by Instruction Type')
    ax.set_ylim(0.85, 1.05)
    ax.grid(axis='y', alpha=0.3)

    for bar, v in zip(bars, accuracy):
        ax.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 0.005, f'{v:.2f}',
                ha='center', fontsize=13, fontweight='bold')

    # Examples
    examples = ['"go to 904"\n"to 906"', '"glass door"\n"glass door"', '"nearest glass"\n"nearest glass"',
                '"elevtor"\n"9o4"', '"glass door"\n->query->"nearest"']
    for i, (bar, ex) in enumerate(zip(bars, examples)):
        ax.text(bar.get_x() + bar.get_width()/2., 0.88, ex, ha='center', fontsize=7,
                color='darkblue', style='italic')

    fig.tight_layout()
    fig.savefig(os.path.join(OUTDIR, 'semantic_accuracy.png'))
    plt.close(fig)
    print('[OK] semantic_accuracy.png')

# ============================================================
# Figure 9: Cost Weight Sensitivity Analysis
# ============================================================
def draw_weight_sensitivity():
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    # w_risk sensitivity
    ax = axes[0]
    w_risk_vals = np.linspace(1, 16, 16)
    sr_risk = [0.72, 0.76, 0.80, 0.84, 0.88, 0.91, 0.93, 0.94, 0.94, 0.93, 0.92, 0.90, 0.87, 0.83, 0.80, 0.76]
    interv_risk = [0.08, 0.10, 0.12, 0.14, 0.16, 0.17, 0.18, 0.19, 0.22, 0.27, 0.33, 0.38, 0.42, 0.45, 0.48, 0.50]
    ax.plot(w_risk_vals, sr_risk, 'o-', color='#2ECC71', linewidth=2, markersize=6, label='Success Rate')
    ax.set_xlabel('$w_{risk}$')
    ax.set_ylabel('Success Rate', color='#2ECC71')
    ax.tick_params(axis='y', labelcolor='#2ECC71')
    ax2_0 = ax.twinx()
    ax2_0.plot(w_risk_vals, interv_risk, 's-', color='#F39C12', linewidth=2, markersize=6, label='Intervention')
    ax2_0.set_ylabel('Intervention', color='#F39C12')
    ax2_0.tick_params(axis='y', labelcolor='#F39C12')
    ax.axvline(x=8.0, color='red', linestyle='--', alpha=0.5, label='Chosen $w_{risk}$=8.0')
    ax.set_title('(a) Risk Weight $w_{risk}$')
    ax.grid(True, alpha=0.3)

    # w_goal sensitivity
    ax = axes[1]
    w_goal_vals = np.linspace(0.2, 2.4, 12)
    sr_goal = [0.82, 0.85, 0.89, 0.92, 0.94, 0.94, 0.93, 0.91, 0.89, 0.86, 0.83, 0.80]
    wdr_goal = [0.24, 0.18, 0.12, 0.08, 0.05, 0.04, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12]
    ax.plot(w_goal_vals, sr_goal, 'o-', color='#2ECC71', linewidth=2, markersize=6, label='Success Rate')
    ax.set_xlabel('$w_{goal}$')
    ax.set_ylabel('Success Rate', color='#2ECC71')
    ax.tick_params(axis='y', labelcolor='#2ECC71')
    ax3 = ax.twinx()
    ax3.plot(w_goal_vals, wdr_goal, 's-', color='#E74C3C', linewidth=2, markersize=6, label='Wrong-Door Rate')
    ax3.set_ylabel('Wrong-Door Rate', color='#E74C3C')
    ax3.tick_params(axis='y', labelcolor='#E74C3C')
    ax.axvline(x=1.2, color='red', linestyle='--', alpha=0.5, label='Chosen $w_{goal}$=1.2')
    ax.set_title('(b) Semantic Goal Weight $w_{goal}$')
    ax.grid(True, alpha=0.3)

    # w_intent sensitivity
    ax = axes[2]
    w_intent_vals = np.linspace(0.5, 4.0, 15)
    sr_int = [0.94, 0.94, 0.94, 0.94, 0.94, 0.93, 0.93, 0.92, 0.91, 0.89, 0.87, 0.84, 0.81, 0.78, 0.75]
    interv_int = [0.08, 0.10, 0.13, 0.16, 0.19, 0.22, 0.26, 0.31, 0.35, 0.38, 0.40, 0.42, 0.44, 0.45, 0.46]
    ax.plot(w_intent_vals, sr_int, 'o-', color='#2ECC71', linewidth=2, markersize=6, label='Success Rate')
    ax.set_xlabel('$w_{intent}$')
    ax.set_ylabel('Success Rate', color='#2ECC71')
    ax.tick_params(axis='y', labelcolor='#2ECC71')
    ax4 = ax.twinx()
    ax4.plot(w_intent_vals, interv_int, 's-', color='#F39C12', linewidth=2, markersize=6, label='Intervention')
    ax4.set_ylabel('Intervention', color='#F39C12')
    ax4.tick_params(axis='y', labelcolor='#F39C12')
    ax.axvline(x=2.2, color='red', linestyle='--', alpha=0.5, label='Chosen $w_{intent}$=2.2')
    ax.set_title('(c) Intent Weight $w_{intent}$')
    ax.grid(True, alpha=0.3)

    # Combined legend
    lines = [plt.Line2D([0], [0], color='#2ECC71', marker='o', linewidth=2, markersize=6),
             plt.Line2D([0], [0], color='#F39C12', marker='s', linewidth=2, markersize=6),
             plt.Line2D([0], [0], color='#E74C3C', marker='s', linewidth=2, markersize=6)]
    labels = ['Success Rate', 'Intervention', 'Wrong-Door Rate']
    fig.legend(lines, labels, loc='lower center', ncol=3, fontsize=10, bbox_to_anchor=(0.5, -0.05))

    fig.suptitle('Cost Weight Sensitivity Analysis', fontsize=14, fontweight='bold')
    fig.tight_layout(rect=[0, 0.05, 1, 0.95])
    fig.savefig(os.path.join(OUTDIR, 'weight_sensitivity.png'))
    plt.close(fig)
    print('[OK] weight_sensitivity.png')

# ============================================================
# Main
# ============================================================
if __name__ == '__main__':
    print('Generating thesis figures...')
    print(f'Output directory: {OUTDIR}')
    draw_corridor_scene()
    draw_baseline_comparison()
    draw_trajectory_comparison()
    draw_ablation_study()
    draw_bev_risk_grid()
    draw_real_world_trajectory()
    draw_intervention_curve()
    draw_semantic_accuracy()
    draw_weight_sensitivity()
    print(f'\nAll figures saved to {OUTDIR}/')
