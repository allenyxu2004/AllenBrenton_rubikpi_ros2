# Simple checklist for tuning Q and R

**R describes uncertainty in the camera measurements. Q describes uncertainty
in the commanded motion.** Start with the camera, then test movement, then run
SLAM with both together.

Change settings in [hw3_slam.yaml](../../robot_control/config/hw3_slam.yaml).
The current noise values are starting guesses, not measured results.

| Parameter | Current value | What it describes |
|---|---:|---|
| `measurement_noise_m` | 0.04 | Base camera noise |
| `range_noise_fraction` | 0.02 | Extra camera noise as tag distance increases |
| `translation_noise` | 0.08 | Uncertainty in forward movement |
| `rotation_noise` | 0.20 | Uncertainty in turning |

## 1. Fix consistent errors first

Check tag size, camera calibration, camera mounting position, and average
driving/turning speeds. If the robot always overshoots by about the same amount,
fix its speed calibration. Noise describes how much results vary between trials.

## 2. Test the camera while stopped: R

- Keep the robot and a tag still.
- Collect about 200 distinct observations at 0.5, 1, 1.5, and 2 meters.
- At each distance, calculate the standard deviation of the raw `forward` and
  `left` measurements. Standard deviation measures how much readings scatter.
- Include valid readings rejected by the filter; using only accepted readings
  can make the camera appear more consistent than it is.

Our model assumes the same noise in both directions:

$$
\text{camera standard deviation}
=\texttt{measurement\_noise\_m}
+\texttt{range\_noise\_fraction}\times\text{distance}.
$$

Choose the two parameters so this line roughly matches the measured scatter
at each distance. If forward and left scatter differ a lot, the equal-noise
assumption is a limitation. Also compare average readings with a tape measure:
consistent but wrong readings suggest a calibration error.

## 3. Repeat forward movements: translation Q

- Repeat the same forward pulse 10-20 times from the same starting pose.
- Use the same pulse length and settling time as SLAM.
- Measure actual travel independently with a ruler or floor markings.
- Subtract predicted travel from actual travel for each trial.
- Calculate the standard deviation of those errors.

For trials with equal moving time T, a starting estimate is:

$$
\texttt{translation\_noise}
\approx\frac{\text{standard deviation of distance errors in meters}}{\sqrt{T}}.
$$

T is commanded moving time in seconds, excluding stopped observation periods.
The estimate assumes your physical measurements are reasonably accurate.

## 4. Repeat turns: rotation Q

Repeat the previous experiment for left and right pivots, measuring angle
instead of distance. Use radians: 90 degrees is about 1.571 radians.

$$
\texttt{rotation\_noise}
\approx\frac{\text{standard deviation of angle errors in radians}}{\sqrt{T}}.
$$

Try several pulse lengths and check heading drift during forward driving too.
The current code uses one rotation-noise setting for all moving actions.

## 5. Check the settings on complete runs

- Run the same route several times, including losing and seeing tags again.
- Compare the estimated map and final robot position with physical measurements.
- Adjust one parameter at a time, by a small amount such as 20%, and repeat.
- Larger Q generally gives camera corrections more influence. Larger R generally
  gives each camera observation less influence.
- Do not judge accuracy only by how smooth the trajectory looks.

The code squares the noise parameters when forming covariance values. Doubling
a noise parameter makes its variance contribution four times larger.

For the report, record the number of trials, measured scatter, chosen settings,
and errors on separate validation runs. Only describe experiments you actually
performed.
