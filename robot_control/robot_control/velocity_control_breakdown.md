# velocity_control.py, Explained Simply

## The big idea

The robot has a list of spots to visit. For each spot it does this:

1. **Spin** until it faces the spot.
2. **Drive** straight for the right amount of time.
3. **Spin** to face the direction the spot asks for.

It can't see where it is. It just says "I drove for 2 seconds at about 0.28 m/s, so I must have moved about 0.56 m," and trusts that.

## The pieces

### The numbers at the top (constants)
Settings we measured by testing the robot:
- How fast it goes when driving straight.
- How fast it spins left, and how fast it spins right.
- Which motor power to use for each move.
- How close is "close enough" (1 cm, about half a degree).

### `Waypoint`
A tiny box holding one target spot: `x`, `y`, and which way to face (`theta`).

### `load_waypoints(path)`
Reads the list of spots from a text file, one `x,y,theta` per line.
It skips blank lines and `#` comments, and it complains if a line is broken.

### `normalize_angle(angle)`
Turns any angle into the shortest way to turn.
"Turn 350 degrees right" becomes "turn 10 degrees left."

### `EstimatedPose`
The robot's guess of where it is (`x`, `y`, `theta`). It starts at the origin facing forward.

- `integrate(speed, turn_rate, elapsed)`: after moving for a bit, update the guess.
  Moving forward changes `x` and `y`, and spinning changes `theta`.
  This is the "kinematic model."

### `run_route(...)`
The part that actually drives. It has four small helpers:

- **`send(left, right)`**: tell the two motors how hard to push.
- **`timed_motion(...)`**: push the motors for a set time, keep updating the guess, then **always stop** at the end.
- **`pause()`**: stand still for a moment so the robot settles.
- **`turn_to(angle)`**: spin toward an angle. Work out how much is left to turn, spin for that long, pause, check again, and repeat until close enough.

Then the main loop:
1. Wait until the motor listener is ready.
2. For each waypoint: turn toward it, drive to it, pause, and repeat until close enough. Then turn to the final direction.
3. When done (or if something goes wrong), send stop commands.

### `main()`
Starts ROS, reads the waypoint file, and calls `run_route`.
If you press Ctrl-C, it stops the motors before quitting.

## The catch

The robot never checks where it really is. If a wheel slips or the battery gets low, the guess drifts away from reality, and nothing fixes it.
