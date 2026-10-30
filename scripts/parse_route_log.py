#!/usr/bin/env python3
"""Extract HW2_REPORT JSON from a saved ROS launch log (Matplotlib for plotting)."""

import argparse
import csv
import json
import math
from pathlib import Path
import re


def read_records(path):
    records = []
    for number, line in enumerate(path.read_text(errors='replace').splitlines(), 1):
        if 'HW2_REPORT ' not in line:
            continue
        payload = line.split('HW2_REPORT ', 1)[1]
        try:
            record, _ = json.JSONDecoder().raw_decode(payload)
        except ValueError as error:
            raise ValueError(f'{path}:{number}: invalid report JSON') from error
        if record.get('version') != 1:
            raise ValueError(f'{path}:{number}: unsupported report version')
        records.append(record)
    return records


def robot_pose(record, landmark, offset):
    """Map optical x-right/z-forward to planar world x/y, yaw CCW."""
    qx, qy, qz, qw = record['quaternion_xyzw']
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    qx, qy, qz, qw = (v / norm for v in (qx, qy, qz, qw))
    nx = 2 * (qx*qz + qw*qy)
    ny = 2 * (qy*qz - qw*qx)
    nz = 1 - 2 * (qx*qx + qy*qy)
    if abs(ny) > math.sin(math.radians(30)):
        return None
    theta = math.radians(landmark['normal_yaw_deg']) - math.atan2(-nx, nz)
    theta = (theta + math.pi) % (2*math.pi) - math.pi
    right, _, forward = record['translation_m']
    forward += offset
    c, s = math.cos(theta), math.sin(theta)
    return (landmark['x_m'] - (c*forward + s*right),
            landmark['y_m'] - (s*forward - c*right), theta)


def write_csv(path, rows, fields):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


def write_trajectory_plot(path, samples, tags):
    """Render equal-scale world coordinates without requiring a display session."""
    try:
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
    except ImportError as error:
        raise RuntimeError(
            'Plotting requires Matplotlib: '
            'python3 -m pip install -r scripts/requirements-plot.txt. '
            'CSV outputs have already been saved.') from error

    figure = Figure(figsize=(8, 8), layout='constrained')
    FigureCanvasAgg(figure)
    axes = figure.subplots()
    axes.scatter([row['x_m'] for row in samples],
                 [row['y_m'] for row in samples],
                 s=22, color='steelblue', label='Estimated robot position', zorder=3)
    if tags:
        axes.scatter([tag['x_m'] for tag in tags.values()],
                     [tag['y_m'] for tag in tags.values()],
                     s=48, marker='s', color='firebrick', label='Measured tags', zorder=4)
        for name, tag in tags.items():
            axes.annotate(name, (tag['x_m'], tag['y_m']),
                          xytext=(7, 7), textcoords='offset points')
    axes.set(title='Visual estimated trajectory', xlabel='World x (m)', ylabel='World y (m)')
    axes.set_aspect('equal', adjustable='box')
    axes.margins(0.15)
    axes.grid(True, alpha=0.3)
    axes.legend()
    # Dots only: do not draw connecting lines through tracking gaps.
    figure.savefig(path)
    figure.savefig(path.with_suffix('.png'), dpi=200)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--tag-map', type=Path, help='Measured tag world poses, JSON')
    parser.add_argument('--interval', type=float, default=0.5)
    parser.add_argument('--output', type=Path, default=Path('route_report'))
    parser.add_argument('--run', help='Run UUID; default is the last run in the file')
    args = parser.parse_args()
    if not math.isfinite(args.interval) or args.interval <= 0:
        parser.error('--interval must be finite and positive')
    records = read_records(args.log)
    if not records:
        parser.error('No HW2_REPORT records found; use logs from the updated controller')
    run = args.run or records[-1]['run']
    records = [r for r in records if r['run'] == run]
    if not records or records[0]['event'] != 'start':
        parser.error('Selected run is missing its start record; capture the entire launch')
    args.output.mkdir(parents=True, exist_ok=True)
    print(f'Run: {run}')
    arrivals = [r for r in records if r['event'] == 'arrival']
    write_csv(args.output / 'waypoint_errors.csv', arrivals,
              ['stage', 'tag', 'elapsed_s', 'source_stamp_s', 'target_distance_m',
               'position_error_m', 'heading_error_deg', 'goal_right_m', 'goal_forward_m'])
    for r in arrivals:
        print(f"Stage {r['stage']} {r['tag']}: {r['position_error_m']:.4f} m, "
              f"{r['heading_error_deg']:+.2f} deg (estimated goal residual)")
    if not any(r['event'] == 'complete' for r in records):
        print('Route completion was not recorded; arrival table may be partial.')
    if not args.tag_map:
        print('Wrote waypoint_errors.csv. Supply --tag-map to produce trajectory outputs.')
        return
    tags = json.loads(args.tag_map.read_text())['tags']
    for name, tag in tags.items():
        if not re.fullmatch(r'tag_\d+', name) or not all(
                math.isfinite(tag[k]) for k in ('x_m', 'y_m', 'normal_yaw_deg')):
            parser.error(f'Invalid map entry: {name}')
    offset = records[0]['camera_forward_m']
    observations = []
    missing = set()
    for r in records:
        if r['event'] == 'complete':
            break
        if r['event'] != 'observation':
            continue
        name = r['observed_tag']
        if name not in tags:
            missing.add(name)
            continue
        pose = robot_pose(r, tags[name], offset)
        if pose is not None:
            observations.append(dict(elapsed_s=r['elapsed_s'],
                                     source_stamp_s=r['source_stamp_s'],
                                     tag=name, x_m=pose[0], y_m=pose[1],
                                     heading_rad=pose[2]))
    observations.sort(key=lambda r: r['source_stamp_s'])
    samples = []
    if observations:
        # Choose the nearest observation to each fixed source-time sample.
        # A missing half-interval stays a gap: never interpolate through tag loss.
        origin = observations[0]['source_stamp_s']
        bins = {}
        for r in observations:
            index = int((r['source_stamp_s']-origin)/args.interval + 0.5)
            distance = abs(r['source_stamp_s'] - (origin + index*args.interval))
            if index not in bins or distance < bins[index][0]:
                bins[index] = (distance, r)
        samples = [dict(row, sample_time_s=index*args.interval)
                   for index, (_, row) in sorted(bins.items())]
    write_csv(args.output / 'trajectory.csv', samples,
              ['sample_time_s', 'elapsed_s', 'source_stamp_s', 'tag', 'x_m', 'y_m', 'heading_rad'])
    if samples:
        try:
            write_trajectory_plot(args.output / 'trajectory.svg', samples, tags)
        except RuntimeError as error:
            parser.exit(1, f'{error}\n')
        print(f'Wrote {len(samples)} trajectory dots, trajectory.svg and trajectory.png to {args.output}')
    else:
        print('No usable mapped observations; no trajectory image generated.')
    if missing:
        print('Unmapped tags skipped: ' + ', '.join(sorted(missing)))
    print('Trajectory uses visual observations only; periods without mapped tags remain gaps.')


if __name__ == '__main__':
    main()
