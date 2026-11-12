"""Make HW3 error plots using smoothed-with-parallel results by default."""

import argparse
import csv
import copy
import json
import math
from pathlib import Path
import re
import textwrap

import numpy as np

from robot_control.slam.evaluate import landmark_errors, world_transform
from robot_control.slam.trajectory import polygon_waypoints


METRES_PER_FOOT = 0.3048
# User-supplied measured sketch: bottom-left origin, +x right, +y up.
SKETCH_FEET = {4: (3, 0), 9: (5, 0), 3: (8, 3), 0: (8, 5),
               8: (3, 8), 2: (5, 8), 7: (0, 3), 5: (0, 5)}
SKETCH_START_FEET_DEG = {'square': (2, 2, 0), 'octagon': (3, 3, 0)}


def next_output_path(path):
    """Choose a version beyond existing figures and their accompanying CSVs."""
    path = Path(path)
    match = re.fullmatch(r'(.+)_v(\d+)', path.stem)
    base = match.group(1) if match else path.stem
    version = int(match.group(2)) if match else 1
    versions = []
    pattern = re.compile(re.escape(base) + r'_v(\d+)(?:$|_)')
    for existing in path.parent.glob(base + '_v*'):
        found = pattern.match(existing.stem)
        if found:
            versions.append(int(found.group(1)))
    if versions:
        version = max(version, max(versions) + 1)
    return path.with_name(f'{base}_v{version}{path.suffix}')


def ideal_path(shape, side_m, start):
    """Transform the commanded polygon into the same frame as the estimates."""
    local = np.array([(0., 0.), *polygon_waypoints(shape, side_m, 1)])
    rotation, translation = world_transform(start)
    return local @ rotation.T + translation


def route_deviation(trajectory, reference, spacing=.01):
    """Arc-length-weighted estimated cross-track distance, sampled at <=1 cm."""
    trajectory, reference = np.asarray(trajectory), np.asarray(reference)
    if spacing <= 0 or not math.isfinite(spacing):
        raise ValueError('Route sampling spacing must be positive and finite')
    points, distances_along = [trajectory[0]], [0.]
    for begin, end in zip(trajectory[:-1], trajectory[1:]):
        length = np.linalg.norm(end - begin)
        if length <= 1e-12:
            continue
        count = max(1, math.ceil(length / spacing))
        fractions = np.arange(1, count + 1) / count
        points.extend(begin + fractions[:, None] * (end - begin))
        distances_along.extend(distances_along[-1] + fractions * length)
    points, distances_along = np.asarray(points), np.asarray(distances_along)
    beginnings, vectors = reference[:-1], np.diff(reference, axis=0)
    lengths_squared = np.sum(vectors**2, axis=1)
    valid = lengths_squared > 1e-20
    beginnings, vectors, lengths_squared = beginnings[valid], vectors[valid], lengths_squared[valid]
    if not len(beginnings):
        raise ValueError('Ideal route needs a nonzero-length segment')
    fractions = np.clip(np.sum((points[:, None] - beginnings) * vectors, axis=2)
                        / lengths_squared, 0, 1)
    projections = beginnings + fractions[:, :, None] * vectors
    residuals = np.linalg.norm(points[:, None] - projections, axis=2)
    closest = np.argmin(residuals, axis=1)
    nearest = projections[np.arange(len(points)), closest]
    errors = residuals[np.arange(len(points)), closest]
    if len(points) == 1:
        weights = np.ones(1)
    else:
        intervals = np.diff(distances_along)
        weights = np.r_[intervals, 0] / 2 + np.r_[0, intervals] / 2
    metrics = dict(mean_m=float(np.average(errors, weights=weights)),
                   rms_m=float(np.sqrt(np.average(errors**2, weights=weights))),
                   sampled_max_m=float(np.max(errors)),
                   estimated_length_m=float(distances_along[-1]))
    return metrics, points, nearest, errors, distances_along


def write_route_metrics(output, panels, reference, baseline, scope):
    """Export route deviation; no estimate is presented as physical tracking."""
    summary = output.with_name(output.stem + '_trajectory_metrics.csv')
    samples = output.with_name(output.stem + '_trajectory_errors.csv')
    with summary.open('w', newline='') as stream, samples.open('w', newline='') as sample_stream:
        writer, sample_writer = csv.writer(stream), csv.writer(sample_stream)
        writer.writerow(['lap', 'baseline', 'scope', 'estimated_mean_cross_track_m',
                         'estimated_rms_cross_track_m', 'sampled_max_cross_track_m',
                         'estimated_path_length_m', 'reference', 'weighting'])
        sample_writer.writerow(['lap', 'baseline', 'scope', 'arc_length_m', 'estimated_x_m',
                                'estimated_y_m', 'nearest_ideal_x_m', 'nearest_ideal_y_m',
                                'estimated_cross_track_m'])
        for lap, (trajectory, _) in panels.items():
            metrics, points, nearest, errors, arc = route_deviation(trajectory, reference)
            writer.writerow([lap, baseline, scope, metrics['mean_m'], metrics['rms_m'],
                             metrics['sampled_max_m'], metrics['estimated_length_m'],
                             'ideal_commanded_polygon', 'arc_length_trapezoid_1cm'])
            for distance, point, target, error in zip(arc, points, nearest, errors):
                sample_writer.writerow([lap, baseline, scope, distance, *point, *target, error])
            print(f'{baseline}, lap {lap}: estimated cross-track mean '
                  f'{100 * metrics["mean_m"]:.2f} cm, RMS {100 * metrics["rms_m"]:.2f} cm')
    print(f'Estimated route metrics: {summary}\nRoute samples: {samples}')


def read_truth(path):
    """Accept measured tag centers with id,x_m,y_m or id,x_ft,y_ft headers."""
    truth = {}
    with Path(path).open() as stream:
        reader = csv.DictReader(stream)
        fields = set(reader.fieldnames or [])
        if {'id', 'x_m', 'y_m'} <= fields:
            x_key, y_key, factor = 'x_m', 'y_m', 1.0
        elif {'id', 'x_ft', 'y_ft'} <= fields:
            x_key, y_key, factor = 'x_ft', 'y_ft', METRES_PER_FOOT
        else:
            raise ValueError('Ground-truth CSV needs id,x_m,y_m or id,x_ft,y_ft')
        for row in reader:
            tag = int(row['id'])
            point = np.array([float(row[x_key]), float(row[y_key])]) * factor
            if tag in truth or not np.all(np.isfinite(point)):
                raise ValueError('Ground truth needs unique IDs and finite coordinates')
            truth[tag] = point
    if not truth:
        raise ValueError('Ground-truth CSV has no landmark rows')
    return truth


def lap_snapshots(events, shape):
    """Select arrival states, excluding later stationary corrections."""
    corners = {'square': 4, 'octagon': 8}[shape]
    snapshots = {}
    for index, event in enumerate(events):
        if event.get('event') == 'arrival':
            corner = event.get('corner', 0)
            if corner > 0 and corner % corners == 0:
                snapshots[corner // corners] = (index, event)
    if not snapshots:
        # An unfinished recording must not be described as a completed lap.
        for index in range(len(events) - 1, -1, -1):
            event = events[index]
            if all(key in event for key in ('state', 'covariance', 'landmarks')):
                return {0: (index, event)}
        raise ValueError('Log has no state/covariance snapshots')
    return snapshots


def transformed_snapshot(events, index, snapshot, start, fitted=None):
    """Rotate a baseline's positions and marginal landmark covariances."""
    rotation, translation = world_transform(start)
    if fitted is None:
        poses = [[0., 0.]]
        poses += [e['state'][:2] for e in events[:index + 1]
                  if e.get('event') == 'estimate']
        poses.append(snapshot['state'][:2])
    else:
        poses = np.asarray(fitted['poses'])[:, :2]
    trajectory = np.array(poses) @ rotation.T + translation
    state = np.asarray(snapshot['state'])
    covariance = np.asarray(snapshot['covariance'])
    landmarks = {}
    for tag, offset in snapshot['landmarks'].items():
        point = rotation @ state[offset:offset + 2] + translation
        block = rotation @ covariance[offset:offset + 2, offset:offset + 2] @ rotation.T
        landmarks[int(tag)] = (point, (block + block.T) / 2)
    return trajectory, landmarks


def fitted_snapshot(result):
    """Adapt fitted landmarks for plotting/errors; only marginal blocks are used."""
    poses = np.asarray(result['poses'], dtype=float)
    if poses.ndim != 2 or poses.shape[1] != 3 or not len(poses) or not np.all(np.isfinite(poses)):
        raise ValueError('Smoothed poses must be finite x/y/heading triples')
    tags = sorted(result['landmarks'], key=int)
    if not tags:
        raise ValueError('Smoothed result has no landmarks')
    state, indices = poses[-1].tolist(), {}
    covariance = np.zeros((3 + 2 * len(tags), 3 + 2 * len(tags)))
    for tag in tags:
        point = np.asarray(result['landmarks'][tag], dtype=float)
        block = np.asarray(result['landmark_covariance_m2'][tag], dtype=float)
        if (point.shape != (2,) or block.shape != (2, 2)
                or not np.all(np.isfinite(point)) or not np.all(np.isfinite(block))):
            raise ValueError('Each fitted landmark needs finite position and 2x2 covariance')
        index = len(state)
        indices[str(tag)] = index
        state.extend(point.tolist())
        covariance[index:index + 2, index:index + 2] = block
    # This adapter is never used for filtering: off-diagonal joint covariance
    # is not available in the saved fit and is not needed for marginal ellipses.
    return dict(state=state, covariance=covariance.tolist(), landmarks=indices)


def smoothed_baseline(events, arrivals, log, laps=None, saved_path=None, parallel=True):
    """Load the full saved fit, or independently refit each requested lap prefix."""
    suffix = '_smoothed_with_parallel.json' if parallel else '_smoothed.json'
    path = saved_path or log.with_name(log.stem + suffix)
    saved = None
    settings = dict(slip_noise=.01, parallel_tags=[4, 9] if parallel else None, parallel_std_deg=2.)
    if path.exists():
        saved = json.loads(path.read_text())
        settings.update(saved['settings'])
        pair = settings['parallel_tags']
        if parallel and (pair is None or len(set(pair)) != 2
                         or not set(map(str, pair)) <= set(saved['landmarks'])):
            raise ValueError('Saved result must contain a parallel prior for two mapped tags')
        if not parallel and pair is not None:
            raise ValueError('Plain smoothing requires a saved fit without a parallel prior')
        if Path(saved.get('source_log', log.name)).name != log.name:
            raise ValueError('Saved smoothed result belongs to another source log')
    elif saved_path is not None:
        raise ValueError(f'Smoothed result not found: {path}')

    def fit(prefix):
        from robot_control.slam.smooth import build_problem, solve
        problem = build_problem(prefix, settings['slip_noise'])
        print(f'Fitting {len(problem["poses"])} poses with parallel tags '
              f'{settings["parallel_tags"]}...', flush=True)
        return solve(problem, settings['parallel_tags'], settings['parallel_std_deg'])

    if laps is None:
        result = saved if saved is not None else fit(events)
        print(f'Baseline: {path if saved is not None else "full-log smoothed fit"}')
        lap = max(arrivals)
        return {lap: (len(events) - 1, fitted_snapshot(result))}, {lap: result}, settings
    snapshots, fitted = {}, {}
    for lap in dict.fromkeys(laps):
        if lap not in arrivals:
            raise ValueError(f'Available lap snapshots: {list(arrivals)}')
        index = arrivals[lap][0]
        # Never truncate a full-run smoothed path and reuse its final map:
        # that would leak later observations into an earlier-lap comparison.
        fitted[lap] = fit(events[:index + 1])
        snapshots[lap] = (index, fitted_snapshot(fitted[lap]))
    return snapshots, fitted, settings


def comparison_ids(snapshots, truth):
    """Use one measured ID set for every lap mean; never zero-fill missing tags."""
    shared = set(truth)
    for _, snapshot in snapshots.values():
        shared &= set(map(int, snapshot['landmarks']))
    return shared


def write_metrics(output, snapshots, truth, start, shared, baseline='online', scope='lap_arrival'):
    """Save all matched landmark errors plus comparable per-lap means in metres."""
    error_path = output.with_name(output.stem + '_landmark_errors.csv')
    summary_path = output.with_name(output.stem + '_lap_errors.csv')
    with error_path.open('w', newline='') as errors, summary_path.open('w', newline='') as summary:
        writer, totals = csv.writer(errors), csv.writer(summary)
        writer.writerow(['lap', 'id', 'estimated_x_m', 'estimated_y_m',
                         'truth_x_m', 'truth_y_m', 'error_m', 'used_in_lap_mean', 'baseline', 'scope'])
        totals.writerow(['lap', 'mean_landmark_error_m', 'comparison_ids',
                         'missing_measured_ids', 'observed_ids_without_truth', 'baseline', 'scope'])
        for lap, (_, snapshot) in snapshots.items():
            rows = sorted(landmark_errors(snapshot, truth, start))
            for row in rows:
                writer.writerow([lap, *row, row[0] in shared, baseline, scope])
            values = [row[-1] for row in rows if row[0] in shared]
            missing = sorted(set(truth) - set(map(int, snapshot['landmarks'])))
            unmeasured = sorted(set(map(int, snapshot['landmarks'])) - set(truth))
            mean = float(np.mean(values)) if values else None
            totals.writerow([lap, mean, ' '.join(map(str, sorted(shared))),
                             ' '.join(map(str, missing)), ' '.join(map(str, unmeasured)), baseline, scope])
            label = f'Lap {lap}' if lap else 'Final recorded state (incomplete route)'
            if scope == 'full_run':
                label = f'Full run ({lap} completed laps)'
            result = f'{100 * mean:.2f} cm' if mean is not None else 'unavailable'
            print(f'{label}: mean landmark error {result}; '
                  f'{len(shared)}/{len(truth)} measured IDs used; '
                  f'missing={missing}; without truth={unmeasured}')
    print(f'Errors: {error_path}\nLap summary: {summary_path}')


def automatic_limits(panels, truth, start, opts):
    """Include both baselines and covariance in shared world-frame bounds."""
    scale = 1 / METRES_PER_FOOT if opts.units == 'ft' else 1.0
    extent = [np.array(start[:2]) * scale]
    extent.extend(opts.ideal_path * scale)
    for trajectory, landmarks in panels.values():
        extent.extend(trajectory * scale)
        for point, covariance in landmarks.values():
            radius = np.sqrt(np.maximum(np.diag(covariance), 0) * 5.991)
            extent.extend([(point - radius) * scale, (point + radius) * scale])
    extent.extend(point * scale for point in truth.values())
    if not opts.ground_truth and not opts.local:
        extent.extend([np.array([0, 0]), np.array([8, 8]) * METRES_PER_FOOT * scale])
    points = np.asarray(extent)
    low, high = points.min(axis=0), points.max(axis=0)
    margin = max(float(np.max(high - low)) * .12, .1 * scale)
    return (low[0] - margin, high[0] + margin, low[1] - margin, high[1] + margin)


def render(events, snapshots, selected, truth, start, shared, shape, opts):
    """Draw one explicitly labeled baseline; estimates are never mixed."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Ellipse, Rectangle

    scale = 1 / METRES_PER_FOOT if opts.units == 'ft' else 1.0
    panels = opts.panels
    limits = opts.bounds
    fig, axes = plt.subplots(1, len(selected), figsize=(6.4 * len(selected), 6.5),
                             squeeze=False, sharex=True, sharey=True)
    for axis, lap in zip(axes[0], selected):
        trajectory, landmarks = panels[lap]
        if not opts.ground_truth and not opts.local:
            axis.add_patch(Rectangle((0, 0), 8 * METRES_PER_FOOT * scale,
                                     8 * METRES_PER_FOOT * scale, fill=False,
                                     edgecolor='.65', linestyle='--', linewidth=1))
        axis.plot(*(opts.ideal_path * scale).T, color='#8a5ab5', linestyle='--',
                  linewidth=1.6, zorder=1)
        axis.plot(*(trajectory * scale).T, color='#2467a6', linewidth=1.3, zorder=2)
        origin = np.array(start[:2]) * scale
        axis.scatter(*origin, marker='s', color='#228344', s=35, zorder=5)
        length = .13 * scale
        axis.annotate('', origin + length * np.array([math.cos(start[2]), math.sin(start[2])]),
                      origin, arrowprops=dict(arrowstyle='->', color='#228344', lw=1.8))
        for tag, (point, covariance) in sorted(landmarks.items()):
            display = point * scale
            values, vectors = np.linalg.eigh(covariance * scale**2)
            width, height = 2 * np.sqrt(np.maximum(values, 0) * 5.991)
            angle = math.degrees(math.atan2(vectors[1, 0], vectors[0, 0]))
            axis.add_patch(Ellipse(display, width, height, angle=angle, fill=False,
                                   edgecolor='#d98214', linewidth=1.2))
            axis.scatter(*display, color='#d98214', s=27, zorder=4)
            axis.annotate(str(tag), display, xytext=(5, 5), textcoords='offset points',
                          color='#a45e00', fontsize=9)
            if tag in truth:
                actual = truth[tag] * scale
                axis.plot([actual[0], display[0]], [actual[1], display[1]],
                          color='#b54343', linewidth=1.1, alpha=.8, zorder=3)
                if opts.label_errors:
                    error = np.linalg.norm(point - truth[tag]) * 100
                    axis.annotate(f'{error:.1f} cm', (actual + display) / 2,
                                  xytext=(4, -10), textcoords='offset points',
                                  color='#b54343', fontsize=8)
        for tag, point in sorted(truth.items()):
            axis.scatter(*(point * scale), color='black', marker='x', s=40, zorder=5)
            axis.annotate(str(tag), point * scale, xytext=(-8, -13),
                          textcoords='offset points', fontsize=8, ha='right')
        title = f'Lap {lap}' if lap else 'Final recorded state — no completed lap'
        if opts.full_run:
            title = f'Full run ({lap} completed {"lap" if lap == 1 else "laps"})'
        if truth:
            rows = landmark_errors(snapshots[lap][1], truth, start)
            values = [row[-1] for row in rows if row[0] in shared]
            result = f'{100 * np.mean(values):.2f} cm' if values else 'unavailable'
            title += f'\nMean landmark error: {result} ({len(shared)}/{len(truth)} IDs)'
        deviation = route_deviation(trajectory, opts.ideal_path)[0]
        title += f'\nEstimated cross-track RMS: {100 * deviation["rms_m"]:.2f} cm'
        axis.set(title=title, xlabel=f'{"Local map" if opts.local else "World"} x ({opts.units})',
                 xlim=limits[:2], ylim=limits[2:])
        axis.set_aspect('equal', adjustable='box')
        axis.grid(alpha=.18)
        for spine in axis.spines.values():
            spine.set_color('.7')
    axes[0, 0].set_ylabel(f'{"Local map" if opts.local else "World"} y ({opts.units})')
    smoothed = opts.baseline != 'online'
    legend = [Line2D([], [], color='#2467a6', label='Smoothed trajectory' if smoothed
                    else 'Online EKF trajectory'),
              Line2D([], [], color='#8a5ab5', linestyle='--', label='Ideal commanded path'),
              Line2D([], [], color='#d98214', marker='o', label='Estimated tag + 95% ellipse',
                     markerfacecolor='#d98214', linestyle='none'),
              Line2D([], [], color='#228344', marker='s', linestyle='none', label='Start / heading')]
    if truth:
        legend += [Line2D([], [], color='black', marker='x', linestyle='none', label='Measured tag'),
                   Line2D([], [], color='#b54343', label='Landmark position error')]
    baseline_label = {'online': 'online EKF-SLAM', 'smoothed': 'offline smoothed (no prior)',
                      'smoothed-with-parallel': 'offline smoothed + parallel prior'}[opts.baseline]
    fig.suptitle(f'{shape.capitalize()} — {baseline_label}', fontsize=15)
    single = len(selected) == 1
    fig.legend(handles=legend, loc='lower center', bbox_to_anchor=(.5, .11 if single else .06),
               ncol=2 if single else 3, frameon=False, fontsize=9)
    note = ('Each panel ends at its lap-arrival snapshot; trajectory is cumulative. '
            'Ellipses show model uncertainty.')
    if opts.full_run and not smoothed:
        note = 'Online estimates over the full recording, including post-route observations.'
    if smoothed:
        scope = ('Full-recording fit, including post-route observations.' if opts.full_run
                 else 'Each panel independently fitted using data through that lap only.')
        pair = opts.fit_settings['parallel_tags']
        prior = (f' Prior: tags {pair[0]}–{pair[1]} parallel to startup heading '
                 f'(σ={opts.fit_settings["parallel_std_deg"]:g}°).' if pair else ' No parallel prior.')
        note = f'{scope}{prior} Ellipses are fitted marginal uncertainties.'
    if truth:
        note += f' Start: ({start[0] / METRES_PER_FOOT:g}, {start[1] / METRES_PER_FOOT:g}) ft, '
        note += f'{math.degrees(start[2]):g}°; alignment treated as exact.'
    fig.text(.5, .01, textwrap.fill(note, width=86 * len(selected)),
             ha='center', va='bottom', fontsize=7.5)
    fig.tight_layout(rect=(0, .28 if single else .17, 1, .89))
    fig.savefig(opts.output, dpi=220)
    plt.close(fig)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--baseline', choices=['smoothed-with-parallel', 'smoothed', 'online'],
                        default='smoothed-with-parallel', help='Default: smoothed-with-parallel')
    parser.add_argument('--both', action='store_true',
                        help='Save matching online and selected smoothing graphs under one version')
    parser.add_argument('--smoothed-json', type=Path,
                        help='Saved fit; default sibling JSON matching the selected smoothing baseline')
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--ground-truth', type=Path, help='Measured CSV; overrides the supplied sketch')
    source.add_argument('--local', action='store_true', help='Local-frame estimates only, without errors')
    parser.add_argument('--start', nargs=3, type=float, metavar=('X_FT', 'Y_FT', 'HEADING_DEG'),
                        help='Measured world start; defaults to sketch start for the logged shape')
    parser.add_argument('--laps', nargs='+', type=int,
                        help='Laps to compare; smoothing independently refits each lap prefix')
    parser.add_argument('--units', choices=['ft', 'm'], default='ft')
    parser.add_argument('--bounds', nargs=4, type=float, metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX'),
                        help='Shared panel limits in display units; use across files for comparisons')
    parser.add_argument('--label-errors', action='store_true', help='Label error lines in centimetres')
    parser.add_argument('--output', type=Path,
                        help='PNG or PDF base name; defaults to graph_v1.png, auto-incremented')
    opts = parser.parse_args(args)
    try:
        if opts.local and opts.start:
            raise ValueError('--local cannot be combined with a world --start')
        if opts.smoothed_json and opts.baseline == 'online':
            raise ValueError('--smoothed-json cannot be used with the online baseline')
        if opts.both and opts.baseline == 'online':
            raise ValueError('--both requires a smoothing baseline to compare against online')
        if opts.ground_truth and opts.start is None:
            raise ValueError('A custom map requires measured --start X_FT Y_FT HEADING_DEG')
        if opts.start and not all(math.isfinite(v) for v in opts.start):
            raise ValueError('--start must contain finite numbers')
        if opts.bounds and (not all(math.isfinite(v) for v in opts.bounds)
                            or opts.bounds[0] >= opts.bounds[1] or opts.bounds[2] >= opts.bounds[3]):
            raise ValueError('--bounds requires finite, increasing x and y limits')
        with opts.log.open() as stream:
            events = [json.loads(line) for line in stream if line.strip()]
        config = next((e for e in events if e.get('event') == 'config'), None)
        shape = config.get('parameters', {}).get('shape') if config else None
        if shape not in SKETCH_START_FEET_DEG:
            raise ValueError('Log config needs shape: square or octagon')
        arrivals = lap_snapshots(events, shape)
        truth = ({} if opts.local else read_truth(opts.ground_truth) if opts.ground_truth
                 else {tag: np.array(point) * METRES_PER_FOOT for tag, point in SKETCH_FEET.items()})
        start_values = (0, 0, 0) if opts.local else (opts.start or SKETCH_START_FEET_DEG[shape])
        start = (start_values[0] * METRES_PER_FOOT, start_values[1] * METRES_PER_FOOT,
                 math.radians(start_values[2]))
        opts.ideal_path = ideal_path(shape, config['parameters']['side_m'], start)
        if not opts.local:
            print(f'Map: {opts.ground_truth or "supplied 8 ft sketch"}; '
                  f'start (ft, ft, degrees): {tuple(start_values)}')
        opts.output = next_output_path(opts.output or opts.log.with_name('graph_v1.png'))
        opts.output.parent.mkdir(parents=True, exist_ok=True)
        baselines = ['online', opts.baseline] if opts.both else [opts.baseline]
        prepared, all_snapshots, all_panels = [], {}, {}
        for baseline in baselines:
            current = copy.copy(opts)
            current.baseline = baseline
            current.fitted, current.fit_settings = {}, {}
            current.full_run = (baseline != 'online' or opts.both) and opts.laps is None
            if baseline != 'online':
                snapshots, current.fitted, current.fit_settings = smoothed_baseline(
                    events, arrivals, opts.log, opts.laps, opts.smoothed_json,
                    parallel=baseline == 'smoothed-with-parallel')
                selected = list(snapshots)
            elif current.full_run:
                index, final = next((i, e) for i, e in reversed(list(enumerate(events)))
                                    if all(k in e for k in ('state', 'covariance', 'landmarks')))
                snapshots = {max(arrivals): (index, final)}
                selected = list(snapshots)
            else:
                snapshots = arrivals
                selected = list(dict.fromkeys(opts.laps or [min(snapshots), max(snapshots)]))
                if any(lap not in snapshots for lap in selected):
                    raise ValueError(f'Available lap snapshots: {list(snapshots)}')
                snapshots = {lap: snapshots[lap] for lap in selected}
            current.panels = {lap: transformed_snapshot(events, *snapshots[lap], start,
                                                        current.fitted.get(lap)) for lap in selected}
            all_snapshots.update({(baseline, lap): value for lap, value in snapshots.items()})
            all_panels.update({(baseline, lap): value for lap, value in current.panels.items()})
            if opts.both:
                label = baseline.replace('-', '_')
                current.output = opts.output.with_name(opts.output.stem + '_' + label + opts.output.suffix)
            prepared.append((current, snapshots, selected))
        shared = comparison_ids(all_snapshots, truth)
        limits = opts.bounds or automatic_limits(all_panels, truth, start, opts)
        for current, snapshots, selected in prepared:
            current.bounds = limits
            render(events, snapshots, selected, truth, start, shared, shape, current)
            scope = 'full_run' if current.full_run else 'lap_arrival'
            if truth:
                write_metrics(current.output, snapshots, truth, start, shared, current.baseline, scope)
            write_route_metrics(current.output, current.panels, opts.ideal_path, current.baseline, scope)
            print(f'Plot: {current.output}')
    except (ValueError, OSError, KeyError, IndexError, ImportError) as error:
        parser.error(str(error))


if __name__ == '__main__':
    main()
