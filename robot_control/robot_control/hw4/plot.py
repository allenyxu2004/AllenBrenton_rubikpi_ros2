"""Plot planned routes and optionally overlay on-robot JSONL estimates."""

import argparse
from dataclasses import asdict
import json
import math

from robot_control.hw4.course import read_course
from robot_control.hw4.planning import CourseGrid


def read_run(path, course, objective):
    with open(path, encoding='utf-8') as stream:
        rows = [json.loads(line) for line in stream]
    config = next((row for row in rows if row['event'] == 'config'), {})
    if (config.get('objective') != objective
            or config.get('course') != json.loads(json.dumps(asdict(course)))):
        raise ValueError(f'{path} uses a different course or objective')
    start = next((row for row in rows if row['event'] == 'run_start'), None)
    if start is None:
        raise ValueError(f'{path} contains no drive run')
    outcome = next((row for row in rows if row['event'] in ('complete', 'fault')), rows[-1])
    xy = [row['pose'][:2] for row in rows if row['event'] == 'estimate'
          and start['elapsed_s'] <= row['elapsed_s'] <= outcome['elapsed_s']]
    if 'pose' in outcome:
        xy.append(outcome['pose'][:2])
    segments = list(zip(xy, xy[1:]))
    distance = sum(math.dist(a, b) for a, b in segments)
    clearance = (min(course.clearance(a, b) for a, b in segments) - course.robot_radius
                 if segments else math.nan)
    duration = outcome.get('run_s', outcome['elapsed_s'] - start['elapsed_s'])
    print(f'  {outcome["event"]} after {duration:.1f} s, '
          f'estimated distance {distance:.2f} m, estimated clearance {clearance:.2f} m')
    return config['waypoints_m'], xy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('course', help='Course YAML')
    parser.add_argument('--output', default='hw4_paths.png', help='Output image')
    parser.add_argument('--distance-log', help='Distance run JSONL')
    parser.add_argument('--safety-log', help='Safety run JSONL')
    args = parser.parse_args()

    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    import numpy as np

    course = read_course(args.course)
    grid = CourseGrid(course)
    x0, y0, x1, y1 = course.bounds
    step = course.resolution
    nx, ny = math.ceil((x1 - x0) / step), math.ceil((y1 - y0) / step)
    clearance = np.full((ny, nx), np.nan)
    for (i, j), point in ((key, point) for key, point in grid.points.items()
                          if isinstance(key, tuple)):
        clearance[j, i] = course.clearance(point, point) - course.robot_radius

    figure, ax = plt.subplots(figsize=(8, 7), layout='constrained')
    ax.set_facecolor('#d7d7d7')
    heatmap = ax.imshow(clearance, origin='lower', interpolation='nearest',
                        extent=(x0, x0 + nx * step, y0, y0 + ny * step), cmap='Greens')
    figure.colorbar(heatmap, ax=ax, label='Clearance from robot footprint (m)')
    for i, (left, bottom, right, top) in enumerate(course.obstacles):
        ax.add_patch(Rectangle((left, bottom), right - left, top - bottom,
                               color='#34343c', label='Obstacle' if i == 0 else None))
    for objective, color, log in [('distance', '#a4194b', args.distance_log),
                                  ('safety', '#104da0', args.safety_log)]:
        plan = grid.plan(objective)
        ax.plot(*zip(*plan.grid_points), color=color, alpha=0.45, linestyle=':')
        ax.plot(*zip(*plan.points), 'o-', color=color, markersize=3,
                label=f'{objective.title()} ({plan.distance:.2f} m)')
        print(f'{objective}: grid {plan.grid_distance:.3f} m, '
              f'waypoints {plan.distance:.3f} m, minimum clearance {plan.clearance:.3f} m')
        if log:
            waypoints, poses = read_run(log, course, objective)
            if waypoints != [list(point) for point in plan.points]:
                ax.plot(*zip(*waypoints), color=color, linestyle='-.',
                        label=f'{objective.title()} recorded plan')
            if poses:
                ax.plot(*zip(*poses), color=color, linestyle='--',
                        label=f'{objective.title()} estimated run')

    ax.scatter(*zip(*course.landmarks.values()), color='black', marker='s',
               label='AprilTag', s=20, clip_on=False)
    for tag, xy in course.landmarks.items():
        ax.annotate(str(tag), xy, xytext=(5, 4), textcoords='offset points')
    for label, xy in [('Start', course.start), ('Goal', course.goal)]:
        ax.annotate(label, xy, xytext=(5, -13), textcoords='offset points')
    ax.set(xlim=(x0, x1), ylim=(y0, y1), xlabel='x (m)', ylabel='y (m)',
           title='HW4 course and planned routes', aspect='equal')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.10), ncol=2)
    figure.savefig(args.output, dpi=180)


if __name__ == '__main__':
    main()
