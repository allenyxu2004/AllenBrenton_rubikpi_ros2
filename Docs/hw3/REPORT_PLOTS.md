# Plot HW3 logs for the report

From the repository root on your laptop, use a Python environment containing
NumPy and Matplotlib. SciPy is also needed when fitting instead of loading a
saved smoothing result. ROS is not needed:

```bash
python3 -m pip install -r scripts/requirements-smooth.txt
PYTHONPATH=robot_control python3 -m robot_control.slam.report RUN.jsonl
```

Figures default to `graph_v1.png` next to the log. Each subsequent invocation
uses the next available version (`graph_v2.png`, `graph_v3.png`, etc.), with
matching versioned error CSVs. Existing figures and CSVs are preserved. Custom
`--output comparison.png` produces `comparison_v1.png`, then `comparison_v2.png`.
The sequence also considers other file formats and leftover CSVs, so exporting
a PDF cannot overwrite the error CSVs belonging to an existing PNG version.

To generate a clearly distinguished pair with the same axes, ideal route,
measured start, and recording scope:

```bash
PYTHONPATH=robot_control python3 -m robot_control.slam.report RUN.jsonl --both
```

This produces `graph_vN_online.png` and
`graph_vN_smoothed_with_parallel.png`, with separate metric CSVs for each.
Both use the full recording, including stationary observations after driving.
Earlier versions are preserved. `--both --baseline smoothed` instead compares
online EKF with plain smoothing **without** a parallel prior. The default paired
baseline preserves the selected smoothing-with-parallel workflow. With
`--laps 1 3`, both baselines use only data through each requested lap; smoothed
prefixes are independently refitted.

Every graph reports **estimated cross-track RMS**: the distance from the
estimated robot path to the nearest point on a finite segment of the ideal
commanded polygon. Each baseline also writes `_trajectory_metrics.csv` (mean,
RMS, sampled maximum, and estimated path length in metres) and
`_trajectory_errors.csv` (estimated points, closest ideal points, and errors).
The estimated polyline is sampled at intervals no longer than 1 cm. Means and
RMS use arc-length weights, so repeated stationary samples and different pose
sampling rates do not bias the metric. The maximum is over the sampled points.

This is estimated route deviation, **not physical tracking error**. No
independent robot trajectory was recorded. Online corrections can create
apparent movement in the plotted estimate and therefore affect the metric.
The closest-route metric does not verify travel direction, timing, or lap
coverage. Landmark position error remains a separate metric against physically
measured tag coordinates.

After building the ROS package, the same command is available as
`ros2 run robot_control hw3_slam_report RUN.jsonl`.

The default map comes from the supplied measured sketch, in feet, with the
bottom-left corner as origin, +x right, and +y up:

| Tag | x (ft) | y (ft) |
|---|---:|---:|
| 4 | 3 | 0 |
| 9 | 5 | 0 |
| 3 | 8 | 3 |
| 0 | 8 | 5 |
| 8 | 3 | 8 |
| 2 | 5 | 8 |
| 7 | 0 | 3 |
| 5 | 0 | 5 |

The sketch presets use square start **(2, 2) ft** and octagon start **(3, 3) ft**,
both facing +x (0 degrees). Check that these match the actual run. Override a
different measured start with `--start X_FT Y_FT HEADING_DEG`. Positive heading
is counterclockwise. This alignment is used offline and treated as exact; it
does not supply prior map information to the online filter.

The dashed purple line is the ideal commanded polygon, generated from the log's
`shape` and `side_m`, rotated and translated by the same measured starting pose.
Repeated laps share the same ideal route. This is a plotting overlay, not a
constraint on smoothing. Black crosses mark measured tags; their labels are
the tag IDs without a `GT` prefix.

```bash
PYTHONPATH=robot_control python3 -m robot_control.slam.report RUN.jsonl \
  --start 2 2 0 --laps 1 3 --bounds -1 10 -1 10 --output comparison.png
```

The default baseline is **smoothed with the parallel prior**, using the sibling
`RUN_smoothed_with_parallel.json`. `--smoothed-json PATH` selects another saved
fit. The trajectory, landmark centers, and covariance all come from that fit.
The default graph is one full-recording panel, including post-route stationary
observations, matching the existing saved smoothed graph. If no saved fit exists,
the script fits the recording with the existing smoother, using tags 4–9 as the
parallel pair, 2-degree prior standard deviation, and 0.01 m/√s slip noise.
Saved fits supply their own recorded settings. Figures disclose the parallel
prior; world landmark coordinates are used for error evaluation, not fitting.

For a lap comparison, use `--laps 1 3` (or `--laps 1 2 3`). Each panel is
**independently smoothed using only events through that lap's arrival**. Later
observations are excluded from earlier fits. The final full-run map is never
reused as the first-lap map. These prefix fits can differ from the existing
full-recording smoothing result. An incomplete route is labeled as such.

`--baseline online` retains the original online EKF mode: by default it compares
the first and last completed laps, excluding post-route observations. Neither
its coordinates nor its covariance are mixed into the smoothed graphs.

`--baseline smoothed` selects the fit without a parallel prior, loading
`RUN_smoothed.json` or fitting it when absent. Add `--both` to either smoothing
baseline to save matching online and smoothed figures with shared axes and
the same version number. Without `--laps`, both cover the full recording;
with `--laps`, each ends at the requested arrival.

Use the same `--bounds` across separate experiments for comparable axes. Bounds
are in display units (feet by default; `--units m` selects metres). Choose bounds
wide enough to include the covariance ellipses. `--label-errors` adds individual
error labels in centimetres. An output ending in `.pdf` saves a vector figure.

For a different measured map, supply a CSV with `id,x_ft,y_ft` or `id,x_m,y_m`
and the measured start:

```bash
PYTHONPATH=robot_control python3 -m robot_control.slam.report RUN.jsonl \
  --ground-truth measured_tags.csv --start 2 2 0 --output report.png
```

`--local` plots estimates in the local map frame without ground truth or errors.
It cannot establish physical accuracy.

Alongside the figure, the script writes:

- `report_landmark_errors.csv`: matched landmark positions and Euclidean errors
  in metres, with baseline and scope (`full_run` or `lap_arrival`).
- `report_lap_errors.csv`: mean position errors, comparison IDs, missing
  measured IDs, observed IDs without measurements, baseline, and scope.
- `report_trajectory_metrics.csv` and `report_trajectory_errors.csv`: estimated
  cross-track distances to the ideal polygon, sampled at at most 1 cm intervals
  and weighted by estimated path length. These compare estimated trajectories
  to the command reference; they do not measure physical tracking accuracy.

All comparison means use the same measured IDs in the compared snapshots, so
missing landmarks cannot artificially improve the comparison. Figure titles
state the number of IDs used. Missing tags are never assigned zero error.
Ellipse size expresses filter uncertainty, while error lines show discrepancy
from the measured map. The world alignment uses the supplied measured start;
it is never optimized to align estimated landmarks to the ground-truth map.
