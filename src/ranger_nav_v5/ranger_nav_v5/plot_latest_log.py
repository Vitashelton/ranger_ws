import argparse
import csv
from pathlib import Path


def latest_csv(log_dir: Path):
    files = sorted(log_dir.glob('*.csv'), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def read_csv(path):
    rows = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows


def col(rows, name, default=0.0):
    out = []
    for r in rows:
        try:
            out.append(float(r.get(name, '') or default))
        except Exception:
            out.append(default)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--log_dir', default='/tmp/rangermini_v5_logs')
    args = ap.parse_args()
    log_dir = Path(args.log_dir)
    path = latest_csv(log_dir)
    if not path:
        print(f'No CSV found in {log_dir}')
        return

    import matplotlib.pyplot as plt

    rows = read_csv(path)
    t = col(rows, 't')
    fig_dir = log_dir / 'figures'
    fig_dir.mkdir(parents=True, exist_ok=True)

    # command comparison
    plt.figure(figsize=(8, 4.6))
    plt.plot(t, col(rows, 'vx_raw'), label='vx raw')
    plt.plot(t, col(rows, 'vx_safe'), label='vx safe')
    plt.plot(t, col(rows, 'vy_raw'), label='vy raw')
    plt.plot(t, col(rows, 'vy_safe'), label='vy safe')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Human command vs shared-control safe output')
    plt.grid(True, alpha=0.3)
    plt.legend()
    out1 = fig_dir / (path.stem + '_cmd_compare.png')
    plt.tight_layout()
    plt.savefig(out1, dpi=180)
    plt.close()

    plt.figure(figsize=(8, 4.2))
    plt.plot(t, col(rows, 'intervention_score'), label='intervention score')
    plt.xlabel('Time (s)')
    plt.ylabel('I(t)')
    plt.title('Shared-control intervention intensity')
    plt.grid(True, alpha=0.3)
    plt.legend()
    out2 = fig_dir / (path.stem + '_intervention.png')
    plt.tight_layout()
    plt.savefig(out2, dpi=180)
    plt.close()

    plt.figure(figsize=(8, 4.2))
    plt.plot(t, col(rows, 'min_distance'), label='minimum obstacle distance')
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.title('Minimum obstacle distance from real-time BEV risk map')
    plt.grid(True, alpha=0.3)
    plt.legend()
    out3 = fig_dir / (path.stem + '_min_distance.png')
    plt.tight_layout()
    plt.savefig(out3, dpi=180)
    plt.close()

    print('Plotted latest log:')
    print(f'  {path}')
    print(f'  {out1}')
    print(f'  {out2}')
    print(f'  {out3}')


if __name__ == '__main__':
    main()
