# HW3 experiment record

Experiments performed November 11, 2026. This records measurements reported
from the robot, not planned experiments. Distances are relative to the robot
center; positive lateral coordinates mean the tag is left of the robot.

## Camera measurement noise (R)

Used tag 4, with a measured 6-inch outer black square (excluding white margin),
matching the configured size of 0.1524 m. Each stationary collection completed
with 200 distinct image timestamps. The sampler used camera forward offset
0.0889 m (3.5 inches), zero lateral offset, and zero mounting yaw. The physical
offset still needs independent verification.

Command pattern:

```bash
ros2 launch robot_control hw3_slam.launch.py drive:=false
# In another sourced terminal:
ros2 run robot_control camera_noise --tag-id 4 --x-ft 0 --y-ft 4 --samples 200
```

Results below use millimetres for standard deviations and biases. Bias is
mean measurement minus the supplied physical position. Combined sigma is
sqrt((forward variance + left variance) / 2), matching the isotropic R model.

| Forward / left position (ft) | Forward std (mm) | Left std (mm) | Combined sigma (mm) | Forward bias (mm) | Left bias (mm) | Run suffix |
|---|---:|---:|---:|---:|---:|---|
| 2 / 0 | 1.043 | 0.023 | 0.738 | +15.40 | -7.13 | 1794434662812035991 |
| 4 / 0 | 2.135 | 0.037 | 1.510 | +33.13 | +25.10 | 1794434717138779984 |
| 6 / 0 | 5.357 | 0.181 | 3.790 | +48.90 | +70.61 | 1794434759304619907 |
| 4 / -1 (corrected label) | 1.779 | 0.430 | 1.294 | +51.66 | +26.89 | 1794434825662468059 |
| 4 / +1 (corrected label) | 2.189 | 0.575 | 1.600 | +36.81 | +10.62 | 1794434854642765988 |
| 4 / 0, repeat setup | 3.699 | 0.085 | 2.616 | +36.74 | +15.03 | 1794435203987206611 |

**Sideways label correction:** the robot was moved left/right relative to a
fixed tag. The first sideways run was entered as `--x-ft 1`, but the tag was
actually right of the robot (`--x-ft -1`). The second was entered as `-1`, but
the tag was actually left (`+1`). The table corrects those ground-truth labels
and lateral biases; raw files retain the original inputs. This correction
assumes the heading was maintained and the offsets were measured accurately.

Files on the robot: `~/.ros/camera_noise/tag_4_<run suffix>/`, containing
`observations.csv` and `summary.json`. The first five datasets were copied to
the laptop under `/tmp/hw3-camera-analysis/`; this temporary location is not
an archival backup. The repeat run was reviewed from its console output only.

Findings for Q6(iv):

- Stationary scatter increases with distance and is much greater in forward
  depth than laterally; equal noise in both directions is a model limitation.
- The first three centered measurements indicate approximately 2.7% forward
  overestimation. The repeated 4-ft setup again showed a +3.67 cm bias versus
  +3.31 cm originally. Bias is much larger than stationary random scatter.
- The first five raw datasets have correlated consecutive forward samples
  (lag-one correlation approximately 0.48–0.92). Distinct timestamps do not
  establish independent measurements.
- Tag size is confirmed. Camera intrinsics/rectification, mounting offsets,
  physical alignment, and distance references remain checks, not established
  causes of the bias. No bias correction has been fitted or applied.
- Selected R settings: `measurement_noise_m=0.01`,
  `range_noise_fraction=0.02`, giving sigma(d)=0.01+0.02d metres and
  R=sigma(d)^2 I. This is a conservative engineering choice informed by
  measured scatter, bias, and between-setup variation, not a fitted covariance
  model or bias correction. At 2/4/6 ft, sigma is 2.22/3.44/4.66 cm. These
  values do not bound every observed error (notably 6-ft lateral bias).
  Within-run covariance alone describes repeatability, not total accuracy.

## Motion noise (Q): completed trials

Rightward drift was reported during forward testing. Forward commands were
changed from L=0.25 / R=0.20 to L=0.24 / R=0.22 (commit `56fd86f`).
These commands were used for the completed forward experiment; no quantitative
before/after lateral-drift measurements were collected in this record.

With only the motor bridge running, the following samplers prompted for reset
before each trial and for physical measurements after stopping:

```bash
ros2 run robot_control translation_noise 3
ros2 run robot_control rotation_noise left
ros2 run robot_control rotation_noise right
```

All three logs were copied from SSH host `276a` and inspected locally.

| Experiment | Trials | Predicted motion | Mean actual motion | Mean error | Sample error std | Noise estimate |
|---|---:|---:|---:|---:|---:|---:|
| Forward | 10 | 0.9144 m (3 ft) | 0.897950 m | -0.016450 m | 0.018350 m | 0.008982 m/sqrt(s) |
| Left turn | 10 | 90.000 degrees | 102.0 degrees | +12.000 degrees | 6.074 degrees | 0.123083 rad/sqrt(s) |
| Right turn | 10 | 90.286 degrees | 111.8 degrees | +21.514 degrees | 7.084 degrees | 0.138225 rad/sqrt(s) |

Noise estimate = sample standard deviation of motion errors / sqrt(commanded
moving time), using metres or radians. Sample variance uses N-1=9. Moving times
exclude stopped observation/settling intervals: forward 4.174008 s, left
0.741750 s, right 0.800000 s. Physical measurement error contributes to scatter;
the exact angle-measurement technique used was not confirmed in this record.

Calibration used:

- Forward: L=0.24, R=0.22, assumed speed 0.219070 m/s, software v0.1.13.
- Left: L=-0.45, R=0.30, assumed rate 2.117690 rad/s, software v0.1.14.
- Right: L=0.45, R=-0.30, assumed rate 1.969739 rad/s, software v0.1.14.
- All: maximum pulse 0.15 s, settling 0.35 s, observation pause 0.25 s.
- Right's predicted angle slightly exceeds 90 because the final pulse has a
  minimum duration of 0.05 s. Bias uses 90.286, not the requested 90.

Entered actual turn magnitudes, in trial order (degrees):

- Left: 105, 94, 95, 110, 100, 100, 106, 110, 95, 105.
- Right: 120, 110, 113, 115, 100, 115, 120, 100, 110, 115.

Both turn directions overshoot systematically. Proportional starting corrections
are `left_rate_rps=2.400049` and `right_rate_rps=2.439098`, calculated as
old rate * mean actual angle / predicted angle. A larger modeled rate commands
shorter turns. These corrections require verification with new trials because
short-pulse acceleration, coasting, and timing can violate proportional scaling.
Suggested provisional noise settings are `translation_noise=0.008982` and one
shared `rotation_noise=0.14` (rounded from the larger direction estimate).
**The proposed turn rates have not been applied.** The YAML retains the tested
turn rates. Selected noise values are now `translation_noise=0.009` (rounded)
and `rotation_noise=0.14`. Estimates above apply to the tested motion durations;
verify after changing calibration. These settings still need validation on
complete SLAM runs.

Robot logs:

- `~/.ros/translation_noise/translation_noise_1794435960891747125.jsonl`
- `~/.ros/rotation_noise/rotation_noise_1794437089845859203.jsonl` (left)
- `~/.ros/rotation_noise/rotation_noise_1794437558886643495.jsonl` (right)

Local copies are under `/tmp/hw3-camera-analysis/` (temporary, not archival).
See [NOISE_TUNING_CHECKLIST.md](NOISE_TUNING_CHECKLIST.md) and
[ROTATION_NOISE_SAMPLER.md](ROTATION_NOISE_SAMPLER.md).

## Route experiments and submission measurements

The user reports reasonably accurate basic square and octagon motion. Final
SLAM map accuracy, ground-truth landmark errors, and repeated-lap improvements
have not been established in this record. Still collect measured tag positions,
each run's starting world pose, square/octagon logs with multiple laps, and a
demonstration video. Use the online EKF trajectory and landmark covariance plots
for the primary Kalman-filter results.
