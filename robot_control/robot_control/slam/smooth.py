"""Generate online and batch-smoothed maps from a detailed HW3 JSONL run.

Each stopped interval has ONE pose, so observations cannot draw fictitious
motion while stationary. Motion factors use integrated prediction records,
not requested pulse durations or EKF-corrected pose differences. The first
pose is fixed at zero; all other poses and landmark centers are fitted jointly.
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.sparse.linalg import splu

from robot_control.slam.evaluate import annotate_landmark_distances, plot_landmark
from robot_control.slam.evaluate import main as plot_online
from robot_control.version import __version__


def wrap(angle):
    """Return signed angles in [-pi, pi)."""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def rotation(angle):
    """Rotate robot-frame vectors into the map frame."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])


def build_problem(events, slip_noise=0.01):
    """Collapse each motion pulse into an edge; retain stationary observations."""
    config = next(e for e in events if e['event'] == 'config')
    parameters = config['parameters']
    if not any(e['event'] == 'prediction' for e in events):
        raise ValueError('Detailed prediction records required (v0.1.4+); '
                         'requested command durations are insufficient.')
    if not math.isfinite(slip_noise) or slip_noise <= 0:
        raise ValueError('slip_noise must be positive and finite')
    for name in ('translation_noise', 'rotation_noise'):
        if not math.isfinite(parameters[name]) or parameters[name] <= 0:
            raise ValueError(f'{name} must be positive and finite')
    poses, times = [np.zeros(3)], [0.0]
    edges, observations = [], []
    delta, covariance, pending = np.zeros(3), np.zeros((3, 3)), False
    endpoint, endpoint_time = None, None
    last_end = -math.inf
    previous_control = None

    def flush():
        nonlocal delta, covariance, pending
        if pending:
            poses.append(np.array(endpoint, dtype=float))
            times.append(endpoint_time)
            edges.append((delta.copy(), np.linalg.inv(np.linalg.cholesky(covariance))))
            delta, covariance, pending = np.zeros(3), np.zeros((3, 3)), False

    for event in events:
        kind = event['event']
        if kind in ('command', 'observation', 'shutdown'):
            flush()
        if kind == 'prediction':
            dt = event['integrated_dt_s']
            begin, end = (event['interval_start_elapsed_s'],
                          event['interval_end_elapsed_s'])
            if (not all(math.isfinite(v) for v in (dt, begin, end))
                    or dt <= 0 or end <= begin or dt > end - begin + 1e-6
                    or begin < last_end - 1e-6):
                raise ValueError('Invalid or overlapping prediction intervals')
            control = event['speed_mps'], event['turn_rate_rps']
            if not all(math.isfinite(v) for v in control) or control == (0, 0):
                raise ValueError('Invalid moving prediction control')
            if pending and control != previous_control:
                flush()
            previous_control, last_end = control, end
            speed, rate = control
            angle = delta[2] + rate * dt / 2
            c, s = math.cos(angle), math.sin(angle)
            F = np.eye(3)
            F[0, 2], F[1, 2] = -speed * s * dt, speed * c * dt
            G = np.array([[c * dt, -speed * s * dt**2 / 2],
                          [s * dt, speed * c * dt**2 / 2], [0, dt]])
            Q = np.diag([parameters['translation_noise']**2,
                         parameters['rotation_noise']**2]) / dt
            covariance = F @ covariance @ F.T + G @ Q @ G.T
            # The online model has singular lateral/pivot translation noise.
            # Explicit slip noise permits small physical deviations rather than
            # enforcing perfect straight lines or zero-displacement pivots.
            covariance[:2, :2] += np.eye(2) * slip_noise**2 * dt
            delta += [speed * c * dt, speed * s * dt, rate * dt]
            endpoint, endpoint_time, pending = event['pose_after'], end, True
        elif kind == 'observation' and event['result'] in ('new', 'updated'):
            z = np.array([event['forward'], event['left']], dtype=float)
            sigma = event.get('measurement_std_m')
            if sigma is None or not math.isfinite(sigma) or sigma <= 0:
                raise ValueError('Each accepted observation needs measurement_std_m')
            if not np.all(np.isfinite(z)):
                raise ValueError('Observation contains nonfinite coordinates')
            observations.append((len(poses) - 1, int(event['tag']), z, sigma))
            if len(poses) > 1:
                poses[-1] = np.array(event['pose_after'], dtype=float)
    flush()
    if not observations:
        raise ValueError('Log has no accepted landmark observations')
    final = next(e for e in reversed(events) if 'landmarks' in e and 'state' in e)
    tags = sorted({tag for _, tag, _, _ in observations})
    landmarks = np.array([final['state'][final['landmarks'][str(tag)]:
                                       final['landmarks'][str(tag)] + 2] for tag in tags])
    poses = np.array(poses)
    if not np.all(np.isfinite(poses)) or not np.all(np.isfinite(landmarks)):
        raise ValueError('Nonfinite initial state')
    return dict(config=config, poses=poses, times=times, edges=edges,
                observations=observations, tags=tags, landmarks=landmarks)


def solve(problem, parallel_tags=None, parallel_std_deg=2.0, max_nfev=500):
    """Fit motion/measurement factors with sparse nonlinear least squares."""
    poses, tags = problem['poses'], problem['tags']
    n = len(poses)
    offset = 3 * (n - 1)
    tag_indices = {tag: i for i, tag in enumerate(tags)}
    edges, observations = problem['edges'], problem['observations']
    if parallel_tags is not None:
        if len(set(parallel_tags)) != 2 or not set(parallel_tags) <= set(tags):
            raise ValueError('--parallel-tags needs two different observed IDs')
        if not math.isfinite(parallel_std_deg) or parallel_std_deg <= 0:
            raise ValueError('--parallel-std-deg must be positive and finite')
        a, b = (tag_indices[tag] for tag in parallel_tags)
        if np.linalg.norm(problem['landmarks'][b] - problem['landmarks'][a]) < 1e-6:
            raise ValueError('Parallel tag centers must be distinct')

    def unpack(vector):
        return (np.vstack([np.zeros(3), vector[:offset].reshape(-1, 3)]),
                vector[offset:].reshape(-1, 2))

    def residual(vector):
        path, landmarks = unpack(vector)
        result = []
        for i, (delta, whitening) in enumerate(edges):
            local = rotation(path[i, 2]).T @ (path[i + 1, :2] - path[i, :2])
            error = np.r_[local - delta[:2],
                          wrap(path[i + 1, 2] - path[i, 2] - delta[2])]
            result.extend(whitening @ error)
        for node, tag, measured, sigma in observations:
            expected = rotation(path[node, 2]).T @ (
                landmarks[tag_indices[tag]] - path[node, :2])
            result.extend((expected - measured) / sigma)
        if parallel_tags is not None:
            direction = landmarks[b] - landmarks[a]
            angle = math.atan2(direction[1], direction[0])
            # Parallel is unoriented: both +x and -x satisfy the constraint.
            error = 0.5 * math.atan2(math.sin(2 * angle), math.cos(2 * angle))
            result.append(error / math.radians(parallel_std_deg))
        return np.array(result)

    rows = 3 * len(edges) + 2 * len(observations) + (parallel_tags is not None)
    pattern = lil_matrix((rows, offset + 2 * len(tags)), dtype=int)

    def mark_pose(row, count, node):
        if node:
            pattern[row:row + count, 3 * (node - 1):3 * node] = 1

    row = 0
    for i in range(len(edges)):
        mark_pose(row, 3, i)
        mark_pose(row, 3, i + 1)
        row += 3
    for node, tag, _, _ in observations:
        mark_pose(row, 2, node)
        column = offset + 2 * tag_indices[tag]
        pattern[row:row + 2, column:column + 2] = 1
        row += 2
    if parallel_tags is not None:
        for index in (a, b):
            pattern[row, offset + 2 * index:offset + 2 * index + 2] = 1
    initial = np.r_[poses[1:].ravel(), problem['landmarks'].ravel()]
    fit = least_squares(residual, initial, jac_sparsity=pattern.tocsr(),
                        x_scale='jac', max_nfev=max_nfev,
                        ftol=1e-7, xtol=1e-7, gtol=1e-7)
    if not fit.success or not np.all(np.isfinite(fit.x)):
        raise ValueError(f'Smoothing failed to converge: {fit.message}')
    # Select landmark columns of the inverse joint information matrix to
    # account for uncertainty in the fitted robot poses.
    information = (fit.jac.T @ fit.jac).tocsc()
    rhs = np.zeros((fit.x.size, 2 * len(tags)))
    rhs[offset:] = np.eye(2 * len(tags))
    try:
        marginal = splu(information).solve(rhs)[offset:]
    except RuntimeError as error:
        raise ValueError('Cannot compute landmark covariance: singular fit') from error
    marginal = (marginal + marginal.T) / 2
    path, landmarks = unpack(fit.x)
    path[:, 2] = wrap(path[:, 2])
    return dict(poses=path.tolist(), landmarks={str(tag): landmarks[i].tolist()
                                               for i, tag in enumerate(tags)},
                landmark_covariance_m2={
                    str(tag): marginal[2 * i:2 * i + 2, 2 * i:2 * i + 2].tolist()
                    for i, tag in enumerate(tags)
                },
                initial_cost=float(np.sum(residual(initial)**2) / 2),
                final_cost=float(fit.cost), evaluations=fit.nfev,
                solver_message=fit.message)


def plot_smoothed(problem, result, output):
    """Plot fitted poses and 95% landmark covariance ellipses."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    scale = 1 / 0.3048
    figure, axis = plt.subplots(figsize=(10, 10))
    path = np.array(result['poses']) * [scale, scale, 1]
    axis.plot(*path[:, :2].T, label='Offline smoothed trajectory')
    axis.scatter(*path[0, :2], marker='s', label='Start')
    positions = {int(tag): np.array(xy) * scale
                 for tag, xy in result['landmarks'].items()}
    for tag, position in positions.items():
        block = np.array(result['landmark_covariance_m2'][str(tag)]) * scale**2
        plot_landmark(axis, tag, position, block)
    annotate_landmark_distances(axis, positions)
    # Include online positions in the bounds so their extent remains comparable.
    all_xy = np.vstack([path[:, :2], list(positions.values())])
    online_poses = problem['online_xy'] * scale
    online_tags = problem['landmarks'] * scale
    all_xy = np.vstack([all_xy, online_poses, online_tags, axis.dataLim.get_points()])
    low, high = all_xy.min(axis=0), all_xy.max(axis=0)
    margin = np.maximum((high - low) * 0.15, 0.5)
    label = 'smoothed with parallel prior' if result['settings']['parallel_tags'] else 'smoothed'
    shape = problem['config']['parameters']['shape']
    axis.set(xlim=(low[0] - margin[0], high[0] + margin[0]),
             ylim=(low[1] - margin[1], high[1] + margin[1]),
             xlabel='x (ft)', ylabel='y (ft)',
             title=f'{shape}: {label} and 95% covariance ellipses')
    axis.set_aspect('equal')
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main(args=None):
    """Generate both PNGs plus a separate, reproducible smoothed JSON result."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output-dir', type=Path, help='Default: beside the input log')
    parser.add_argument('--slip-noise', type=float, default=0.01,
                        help='Extra translation noise in m/sqrt(s), default 0.01')
    parser.add_argument('--parallel-tags', nargs=2, type=int, metavar=('ID_A', 'ID_B'),
                        help='Optional physical prior: this tag pair parallels startup heading')
    parser.add_argument('--parallel-std-deg', type=float, default=2.0,
                        help='Parallel prior standard deviation in degrees, default 2')
    parser.add_argument('--all-plots', action='store_true',
                        help='Generate both smoothed variants and the online plot (default pair: 4 9)')
    opts = parser.parse_args(args)
    directory = opts.output_dir or opts.log.parent
    prefix = directory / opts.log.stem
    online = prefix.with_name(prefix.name + '_online.png')
    try:
        with opts.log.open() as stream:
            events = [json.loads(line) for line in stream if line.strip()]
        if opts.all_plots:
            directory.mkdir(parents=True, exist_ok=True)
            plot_online([str(opts.log), '--output', str(online)])
            if not any(e['event'] == 'prediction' for e in events):
                print('Saved online plot only: detailed motion predictions are missing.')
                return
            if not any(e['event'] == 'observation' and e['result'] in ('new', 'updated')
                       for e in events):
                print('Saved online plot only: no accepted tag observations.')
                return
        problem = build_problem(events, opts.slip_noise)
        problem['online_xy'] = np.array([e['state'][:2] for e in events
                                         if e['event'] == 'estimate'])
        if len(problem['online_xy']) == 0:
            raise ValueError('Log has no online estimates')
        print(f'Smoothing {len(problem["poses"])} poses, '
              f'{len(problem["tags"])} tags, '
              f'{len(problem["observations"])} observations...', flush=True)
        pairs = [opts.parallel_tags]
        if opts.all_plots:
            pair = opts.parallel_tags or [4, 9]
            pairs = [None]
            if set(pair) <= set(problem['tags']):
                pairs.append(pair)
            else:
                print(f'Skipping parallel fit: need observations of both tags {pair}.')
        for pair in pairs:
            result = solve(problem, pair, opts.parallel_std_deg)
            directory.mkdir(parents=True, exist_ok=True)
            if not opts.all_plots:
                plot_online([str(opts.log), '--output', str(online)])
            save_smoothed(problem, result, prefix, opts, pair)
    except (ValueError, KeyError, StopIteration, OSError, np.linalg.LinAlgError) as error:
        parser.error(str(error) or 'Incomplete log schema')


def save_smoothed(problem, result, prefix, opts, parallel_tags):
    """Write one fitted result and its plot."""
    suffix = '_smoothed_with_parallel' if parallel_tags else '_smoothed'
    smoothed = prefix.with_name(prefix.name + suffix + '.png')
    data = prefix.with_name(prefix.name + suffix + '.json')
    result.update(source_log=str(opts.log.resolve()), software_version=__version__,
                  source_software_version=problem['config'].get('software_version'),
                  pose_elapsed_s=problem['times'],
                  settings=dict(slip_noise=opts.slip_noise,
                                parallel_tags=parallel_tags,
                                parallel_std_deg=opts.parallel_std_deg),
                  assumptions=['Startup pose fixed at (0,0,0).',
                               'One pose per stationary interval; tags are stationary.',
                               'Motion from integrated predictions, not wheel measurements.',
                               'Accepted new/updated observations only; independent Gaussian noise.',
                               'No forced square, endpoint closure, or ground-truth positions.',
                               'Landmark covariance is linearized at the fitted solution.'])
    plot_smoothed(problem, result, smoothed)
    data.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(f'Weighted cost: {result["initial_cost"]:.3f} -> {result["final_cost"]:.3f}')
    print(f'Smoothed plot: {smoothed}\nSmoothed data: {data}')


if __name__ == '__main__':
    main()
