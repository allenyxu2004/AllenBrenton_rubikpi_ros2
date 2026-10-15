# Our Robot's Kinematic Model, Explained Simply

## What is a kinematic model?

It's the rules for how the robot moves. You tell it how to push its wheels, and the rules tell you where the robot ends up.

## The robot's three tricks

Our robot only knows three moves. For each one, we always use the same wheel power, so we know exactly what it does because we measured it.

| Trick | What the wheels do | What the robot does |
|---|---|---|
| **Go forward** | Both wheels push forward (left 0.36, right 0.2) | Moves straight at about 0.28 meters per second |
| **Spin left** | Left wheel back, right wheel forward | Turns on the spot counter-clockwise, about 2.1 radians per second |
| **Spin right** | Left wheel forward, right wheel back | Turns on the spot clockwise, about 2.1 radians per second |

The left and right wheel powers are not equal when going forward. One motor is a bit stronger than the other, so we give it less power to make the robot go straight.

## Two words you need

- **v** is how fast the robot moves forward. "Zoom!" is a big v.
- **ω** (omega) is how fast it turns. Zero means no turning.

Each trick has a v and an ω:
- Go forward: v = 0.28, ω = 0
- Spin left: v = 0, ω = +2.1
- Spin right: v = 0, ω = -2.1

## How do we know where the robot is?

The robot has no eyes. It can't look around to see where it is. So it keeps a diary of where it thinks it is, called the **pose**:

- **x**: how far across the room
- **y**: how far up the room
- **θ** (theta): which way its nose points

Every 50 milliseconds, the robot updates its diary:

```
x     = x + v · cos(θ) · time
y     = y + v · sin(θ) · time
θ     = θ + ω · time
```

In plain words:
- If it went forward, move x and y a little in the direction the nose points.
- If it spun, turn the nose a little.

## How does it know how long to do each trick?

Because the power is always the same, the only thing that changes is **time**:

```
Drive 1 meter?     Go forward for  1 / 0.28  = about 3.6 seconds.
Turn 90 degrees?   Spin for  1.57 / 2.1  = about 0.74 seconds.
```

Longer trip means longer time. That's the whole trick.

## How it visits a spot

For each waypoint:

1. **Spin** until the nose points at the spot.
2. **Go forward** for the right amount of time.
3. **Spin** to face the direction the waypoint asks for.

## What we are assuming

These are the "pretend" rules. If any of them are wrong, the robot will end up a bit off.

- The robot can't slide sideways. It only rolls forward or spins.
- Each trick is always the same speed. It doesn't speed up or slow down gradually.
- Twice as long means twice as far. This is least true for very short moves, because motors take a moment to start.
- The numbers were measured on this robot, this floor and this battery. A different floor or a low battery changes them.
- The robot never checks where it really is. Small mistakes add up and are never fixed.

## Kinematics vs. calibration

- **Kinematics** is the rules above (the equations and the timing).
- **Calibration** is how we got the numbers: we ran the robot, measured how far it went and how much it turned, and wrote down 0.28 m/s and 2.1 rad/s.

## Where this lives in the code

| Idea | Code |
|---|---|
| The diary update | `EstimatedPose.integrate` in `velocity_control.py` |
| The measured numbers | Constants at the top of `velocity_control.py` |
| Timing and the three tricks | `turn_to` and `run_route` in `velocity_control.py` |
