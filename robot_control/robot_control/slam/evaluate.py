"""Plot a run and evaluate against independently measured landmark centers."""

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np


# Pairs in the current physical layout; edit here if the tags are rearranged.
DISTANCE_PAIRS = ((7, 5), (4, 9), (3, 0), (8, 2))


def annotate_landmark_distances(axis, positions):
    """Label estimated center-to-center distances; positions are in feet."""
    legend_added = False
    for first, second in DISTANCE_PAIRS:
        # Partial maps and older runs may not contain every requested ID.
        if first not in positions or second not in positions:
            continue
        start, end = positions[first], positions[second]
        distance = float(np.linalg.norm(end - start))
        axis.plot([start[0], end[0]], [start[1], end[1]], '--',
                  color='seagreen', linewidth=1.4,
                  label=None if legend_added else 'Estimated tag-pair distance')
        legend_added = True
        axis.annotate(f'{first}–{second}: {distance:.2f} ft', (start + end) / 2,
                      xytext=(8, 8), textcoords='offset points', fontsize=9,
                      color='darkgreen', zorder=5,
                      bbox=dict(boxstyle='round,pad=.25', facecolor='white',
                                edgecolor='seagreen', alpha=0.95))
        print(f'Estimated tag distance {first}–{second}: {distance:.2f} ft')


def world_transform(start):
    """Measured startup robot pose maps the arbitrary SLAM frame to the room."""
    x, y, theta = start
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]]), np.array([x, y])


def landmark_errors(snapshot, truth, start):
    """Return errors for observed IDs that have a ground-truth measurement."""
    rotation, translation = world_transform(start)
    state = np.array(snapshot['state'])
    rows = []
    for tag, index in snapshot['landmarks'].items():
        if int(tag) in truth:
            estimated = rotation @ state[index:index + 2] + translation
            actual = truth[int(tag)]
            rows.append((int(tag), *estimated, *actual,
                         float(np.linalg.norm(estimated - actual))))
    return rows


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--ground-truth', type=Path, help='CSV header: id,x_m,y_m')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X', 'Y', 'YAW'),
                        help='Measured startup robot pose in world metres/radians')
    parser.add_argument('--output', type=Path, help='PNG output; default beside log')
    opts = parser.parse_args(args)
    if opts.ground_truth and opts.start is None:
        parser.error('--ground-truth requires independently measured --start X Y YAW')
    if opts.start is not None and not all(math.isfinite(v) for v in opts.start):
        parser.error('--start must be finite')
    with opts.log.open() as stream:
        events = [json.loads(line) for line in stream if line.strip()]
    estimates = [event for event in events if event['event'] == 'estimate']
    if not estimates:
        parser.error('Log has no estimates')
    # Shutdown includes observations that arrived after the last periodic record.
    final = next((e for e in reversed(events) if e['event'] == 'shutdown'), estimates[-1])
    run_config = next(e for e in events if e['event'] == 'config')
    config = run_config['parameters']
    print(f'Run software version: {run_config.get("software_version", "unversioned (older log)")}')
    truth = {}
    if opts.ground_truth:
        with opts.ground_truth.open() as stream:
            for row in csv.DictReader(stream):
                point = np.array([float(row['x_m']), float(row['y_m'])])
                if not np.all(np.isfinite(point)) or int(row['id']) in truth:
                    parser.error('Ground truth requires finite positions and unique IDs')
                truth[int(row['id'])] = point
    output = opts.output or opts.log.with_suffix('.png')
    output.parent.mkdir(parents=True, exist_ok=True)
    start = opts.start or (0, 0, 0)
    rotation, translation = world_transform(start)

    # Use a noninteractive backend so this also works over plain SSH.
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Ellipse

    # Convert only the display: inputs and quantitative errors remain in metres.
    feet_per_metre = 1 / 0.3048
    figure, axis = plt.subplots(figsize=(10, 10))
    trajectory = (np.array([e['state'][:2] for e in estimates]) @ rotation.T
                  + translation) * feet_per_metre
    axis.plot(*trajectory.T, label='Estimated robot trajectory')
    axis.scatter(*trajectory[0], marker='s', label='Start')
    state, covariance = np.array(final['state']), np.array(final['covariance'])
    landmark_positions = {}
    for tag, index in final['landmarks'].items():
        position = (rotation @ state[index:index + 2] + translation) * feet_per_metre
        landmark_positions[int(tag)] = position
        block = rotation @ covariance[index:index + 2, index:index + 2] @ rotation.T
        block *= feet_per_metre**2  # Covariance has squared length units.
        values, vectors = np.linalg.eigh(block)
        # 95% joint confidence region for a 2D Gaussian: chi-square(2)=5.991.
        width, height = 2 * np.sqrt(np.maximum(values, 0) * 5.991)
        angle = math.degrees(math.atan2(vectors[1, 0], vectors[0, 0]))
        axis.scatter(*position, color='tab:orange')
        axis.annotate(str(tag), position)
        axis.add_patch(Ellipse(position, width, height, angle=angle,
                               fill=False, edgecolor='tab:orange'))
    annotate_landmark_distances(axis, landmark_positions)
    if truth:
        axis.scatter(*(np.array(list(truth.values())) * feet_per_metre).T,
                     marker='x', color='black',
                     label='Measured landmark centers')
        for tag, position in truth.items():
            axis.annotate(f'GT {tag}', position * feet_per_metre)
        rows = landmark_errors(final, truth, start)
        with output.with_suffix('.csv').open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['id', 'estimated_x_m', 'estimated_y_m',
                             'truth_x_m', 'truth_y_m', 'error_m'])
            writer.writerows(rows)
        if rows:
            print(f'Final mean landmark error: {np.mean([r[-1] for r in rows]):.4f} m '
                  f'({len(rows)}/{len(truth)} measured landmarks observed)')
        print('Unobserved measured IDs:', sorted(set(truth) - {r[0] for r in rows}))
        print('Observed IDs without ground truth:',
              sorted(set(map(int, final['landmarks'])) - set(truth)))
        corners = 4 if config['shape'] == 'square' else 8
        for event in events:
            if event['event'] == 'arrival' and event['corner'] % corners == 0:
                errors = landmark_errors(event, truth, start)
                if errors:
                    print(f'Lap {event["corner"] // corners}: '
                          f'mean error {np.mean([r[-1] for r in errors]):.4f} m '
                          f'over {len(errors)} landmarks')
    axis.set(xlabel='x (ft)', ylabel='y (ft)',
             title=f'{config["shape"]}: landmark estimates and 95% covariance ellipses')
    axis.set_aspect('equal')
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)
    print(f'Plot: {output}')


if __name__ == '__main__':
    main()
