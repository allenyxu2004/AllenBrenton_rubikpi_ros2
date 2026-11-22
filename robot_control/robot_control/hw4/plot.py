"""Plot both HW4 roadmap representations and export plans/run metrics without ROS."""

import argparse
from dataclasses import asdict
import csv
import json
import math
from pathlib import Path

from robot_control.hw4.course import read_course
from robot_control.hw4.planning import plan_course
from robot_control.version import __version__


def read_run(path, course, objective):
    """Read new plan events and legacy grid logs; never infer successful arrival."""
    lines = Path(path).read_text(encoding='utf-8').splitlines()
    rows = []
    for index, line in enumerate(lines):
        if not line.strip():
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            if index != len(lines) - 1:
                raise
            print(f'Ignoring interrupted final JSONL record in {path}')
    config = next((r for r in rows if r.get('event') == 'config'), {})
    expected = json.loads(json.dumps(asdict(course)))
    recorded = config.get('course', {})
    if (config.get('objective') != objective or not recorded
            or any(expected[key] != value for key, value in recorded.items() if key in expected)):
        raise ValueError(f'{path} uses a different course or objective')
    plans = [dict(generation=r['generation'], points=r['points']) for r in rows if r.get('event') == 'plan']
    if not plans and config.get('waypoints_m'):
        plans = [dict(generation=0, points=config['waypoints_m'])]
    start = next((r for r in rows if r.get('event') == 'run_start'), None)
    outcome = next((r for r in rows if r.get('event') in ('complete', 'fault')), None)
    status = outcome['event'] if outcome else ('incomplete' if start else 'stationary')
    end = outcome or (rows[-1] if rows else {})
    xy = [r['pose'][:2] for r in rows if r.get('event') == 'estimate' and start
          and start['elapsed_s'] <= r['elapsed_s'] <= end.get('elapsed_s', math.inf)]
    if start and 'pose' in end:
        xy.append(end['pose'][:2])
    segments = list(zip(xy, xy[1:]))
    metrics = dict(status=status, run_s=(end.get('elapsed_s', 0) - start['elapsed_s']) if start else None,
                   estimated_distance_m=sum(math.dist(a, b) for a, b in segments) if segments else None,
                   estimated_body_clearance_m=min(course.clearance(a, b) - course.robot_radius
                                                  for a, b in segments) if segments else None,
                   final_estimated_residual_m=math.dist(xy[-1], course.goal) if xy else None,
                   replans=max(0, len(plans) - 1),
                   fault_reason=outcome.get('reason', '') if outcome else '')
    return dict(plans=plans, xy=xy, metrics=metrics)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('course', help='Course YAML')
    parser.add_argument('--output', default='hw4_paths.png', help='Figure plus companion .json/.csv files')
    parser.add_argument('--distance-log')
    parser.add_argument('--safety-log')
    parser.add_argument('--overwrite', action='store_true', help='Replace existing outputs explicitly')
    args = parser.parse_args(argv)
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    from matplotlib.patches import Rectangle

    course = read_course(args.course)
    output = Path(args.output)
    if output.suffix.lower() not in ('.png', '.svg', '.pdf'):
        parser.error('Output must be PNG, SVG or PDF')
    targets = [output, output.with_suffix('.json'), output.with_suffix('.csv')]
    if not args.overwrite and any(p.exists() for p in targets):
        parser.error('Output exists; use a new name or --overwrite')
    figure, axes = plt.subplots(1, 2, figsize=(13, 7), layout='constrained')
    payload = dict(software_version=__version__, course=asdict(course), plans={}, runs={})
    metrics = []
    for ax, objective, color, log in zip(axes, ('safety', 'distance'), ('#104da0', '#a4194b'),
                                         (args.safety_log, args.distance_log)):
        plan = plan_course(course, objective)
        payload['plans'][objective] = asdict(plan)
        roadmap = [(a, b) for a, b, kind in plan.graph_edges if kind != 'connector']
        connectors = [(a, b) for a, b, kind in plan.graph_edges if kind == 'connector']
        ax.add_collection(LineCollection(roadmap, colors=color, linewidths=.5, alpha=.22,
                                         label='Roadmap edges'))
        if connectors:
            ax.add_collection(LineCollection(connectors, colors='#8a99ab', linewidths=.3, alpha=.06))
        ax.add_collection(LineCollection(course.boundary_segments, colors='#c77715',
                                         linewidths=1, linestyles='dashed', label='Reserved free boundary'))
        if plan.sites:
            ax.scatter(*zip(*plan.sites), s=2, color='#c77715', alpha=.5, label='Boundary sites')
        for i, (left, bottom, right, top) in enumerate(course.obstacles):
            ax.add_patch(Rectangle((left, bottom), right - left, top - bottom, color='#34343c',
                                   label='Physical obstacle' if i == 0 else None))
        ax.plot(*zip(*plan.points), 'o-', color=color, markersize=4, linewidth=2,
                label=f'Planned route ({plan.distance:.2f} m)')
        row = dict(objective=objective, planner=plan.planner, planned_distance_m=plan.distance,
                   planned_body_clearance_m=plan.clearance, residual_clearance_m=plan.residual_clearance,
                   build_s=plan.build_s, search_s=plan.search_s, status='no_run', run_s=None,
                   estimated_distance_m=None, estimated_body_clearance_m=None,
                   final_estimated_residual_m=None, replans=0, fault_reason='')
        if log:
            run = read_run(log, course, objective)
            payload['runs'][objective] = run
            row.update(run['metrics'])
            for active in run['plans']:
                ax.plot(*zip(*active['points']), color=color, linestyle='-.', alpha=.6)
            if run['xy']:
                ax.plot(*zip(*run['xy']), color='#202b38', linestyle='--', label='Online estimates')
        metrics.append(row)
        ax.scatter(*zip(*course.landmarks.values()), color='black', marker='s', s=13, label='AprilTag')
        for tag, xy in course.landmarks.items():
            ax.annotate(str(tag), xy, xytext=(4, 4), textcoords='offset points', fontsize=7)
        for label, xy in [('Start', course.start), ('Goal', course.goal)]:
            ax.annotate(label, xy, xytext=(5, -12), textcoords='offset points', fontsize=9)
        ax.arrow(*course.start, .16 * math.cos(course.heading), .16 * math.sin(course.heading),
                 width=.005, head_width=.04, color='#1565c0', length_includes_head=True)
        x0, y0, x1, y1 = course.bounds
        ax.set(xlim=(x0, x1), ylim=(y0, y1), xlabel='x (m)', ylabel='y (m)', aspect='equal',
               title=f'{objective.title()}: {plan.planner.title()}\n'
                     f'Body clearance {plan.clearance:.3f} m; reserve {course.padding:.3f} m')
        ax.legend(loc='upper center', bbox_to_anchor=(.5, -.12), fontsize=8, ncol=2)
        print(f'{objective}: {plan.distance:.3f} m; body clearance {plan.clearance:.3f} m; '
              f'{len(plan.graph_nodes)} vertices; {plan.build_s + plan.search_s:.3f} s; {row["status"]}')
    figure.suptitle('HW4 roadmaps | radius + margin + waypoint tolerance')
    # Exclusive files by default; no accidental overwrite of experiment evidence.
    mode = 'w' if args.overwrite else 'x'
    with targets[1].open(mode, encoding='utf-8') as stream:
        json.dump(payload, stream, indent=2, allow_nan=False)
    with targets[2].open(mode, encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(metrics[0]))
        writer.writeheader()
        writer.writerows(metrics)
    with output.open('wb' if args.overwrite else 'xb') as stream:
        figure.savefig(stream, format=output.suffix[1:], dpi=180)
    plt.close(figure)
    print('Estimated travel/clearance include EKF correction jumps; physical measurements remain separate.')


if __name__ == '__main__':
    main()
