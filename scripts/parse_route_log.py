#!/usr/bin/env python3
"""Extract HW2_REPORT JSON from a saved ROS launch log (standard library only)."""

import argparse
import csv
import html
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


def write_svg(path, samples, tags):
    points = [(r['x_m'], r['y_m']) for r in samples]
    points += [(t['x_m'], t['y_m']) for t in tags.values()]
    xs, ys = zip(*points)
    xmin, xmax, ymin, ymax = min(xs), max(xs), min(ys), max(ys)
    scale = 600 / max(xmax-xmin, ymax-ymin, 1)
    def xy(x, y):
        return 70 + (x-xmin)*scale, 710 - (y-ymin)*scale
    elements = ['<svg xmlns="http://www.w3.org/2000/svg" width="800" height="800">',
                '<rect width="800" height="800" fill="white"/>',
                '<text x="30" y="30" font-size="18">Visual estimated trajectory (metres; +x right, +y up)</text>',
                '<text x="30" y="55">Blue dots: sampled robot positions; red squares: measured tags</text>']
    for row in samples:
        x, y = xy(row['x_m'], row['y_m'])
        label = f"t={row['elapsed_s']:.2f}s x={row['x_m']:.3f} y={row['y_m']:.3f}"
        elements.append(f'<circle cx="{x}" cy="{y}" r="4" fill="steelblue"><title>{label}</title></circle>')
    for name, tag in tags.items():
        x, y = xy(tag['x_m'], tag['y_m'])
        elements.append(f'<rect x="{x-4}" y="{y-4}" width="8" height="8" fill="red"/>')
        elements.append(f'<text x="{x+8}" y="{y}">{html.escape(name)}</text>')
    # Coordinate ticks retain the same scale on both axes.
    for i in range(6):
        xval = xmin + (xmax-xmin)*i/5
        yval = ymin + (ymax-ymin)*i/5
        x, _ = xy(xval, ymin)
        _, y = xy(xmin, yval)
        elements.append(f'<text x="{x}" y="735" font-size="12">{xval:.2f}</text>')
        elements.append(f'<text x="10" y="{y}" font-size="12">{yval:.2f}</text>')
    elements.append('</svg>')
    path.write_text('\n'.join(elements))


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
        write_svg(args.output / 'trajectory.svg', samples, tags)
        print(f'Wrote {len(samples)} trajectory dots and trajectory.svg to {args.output}')
    else:
        print('No usable mapped observations; no trajectory image generated.')
    if missing:
        print('Unmapped tags skipped: ' + ', '.join(sorted(missing)))
    print('Trajectory uses visual observations only; periods without mapped tags remain gaps.')


if __name__ == '__main__':
    main()
