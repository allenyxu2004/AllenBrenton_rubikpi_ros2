# Robot Setup & Calibration Guide

## Network & SSH Setup

### Connect to Wi-Fi

```bash
sudo nmcli dev wifi connect "WIFI_Name" password "password"
```

### View Wi-Fi List

Rescan and display available networks:

```bash
sudo nmcli device wifi rescan
sudo nmcli dev wifi show
```

### Check IP Address (Hostname)

```bash
hostname -I
```

### Connect via SSH

```bash
ssh ubuntu@xxx.xxx.xxx.xxx
password: csefa2026
```

### Cloning Repo:
cd ros2_ws/AllenBrenton_rubikpi_ros2
git pull


---

## Calibration Scripts (`scripts/`)

> [!IMPORTANT]
> Both calibration scripts communicate directly over `/dev/ttyUSB0` at `115200` baud. Do **not** run these scripts while the `motor_control` ROS 2 node is running, as both attempt to open the same serial port.

### 1. Straight-Line / Timed Motion (`scripts/drive.py`)

Sends raw left (`L`) and right (`R`) wheel command values (`{"T": 1, "L": ..., "R": ...}`) for a specified duration, then stops the motors.

#### Usage

```bash
python3 scripts/drive.py [L] [R] [duration] [-t TIMES] [-p PORT]
```

#### Arguments & Options

| Argument / Flag | Default | Description |
| :--- | :--- | :--- |
| `L` | `0.2` | Left wheel speed command in `[-0.5, 0.5]` |
| `R` | `0.2` | Right wheel speed command in `[-0.5, 0.5]` |
| `duration` | `2.0` | Motion duration in seconds (`> 0`) |
| `-t`, `--times` | `1` | Number of consecutive trials to run (0.5 s pause between runs) |
| `-p`, `--port` | `/dev/ttyUSB0` | Serial port device path |

#### Examples

Run with default settings (`L=0.2`, `R=0.2` for `2.0` seconds):

```bash
python3 scripts/drive.py
```

Drive forward at `L=0.3`, `R=0.3` for `3.5` seconds across `3` trials:

```bash
python3 scripts/drive.py 0.3 0.3 3.5 -t 3
```

Drive backward at `L=-0.2`, `R=-0.2` for `2.0` seconds:

```bash
python3 scripts/drive.py -0.2 -0.2 2.0
```

---

### 2. Pivot Turn Calibration (`scripts/turn.py`)

Tests timed in-place pivot turns (`left` or `right`) by continuously streaming motor commands at 20 Hz for the target duration. Prompts before each trial so you can mark the starting heading, and optionally computes the average angular velocity (`deg/s`) from your measured angle.

#### Usage

```bash
python3 scripts/turn.py {left,right} [duration] [--left-speed SPEED] [--right-speed SPEED] [-t TIMES] [-p PORT]
```

#### Arguments & Options

| Argument / Flag | Default | Description |
| :--- | :--- | :--- |
| `direction` | *(required)* | Pivot direction: `left` (`-L, +R`) or `right` (`+L, -R`) |
| `duration` | `0.5` | Turn duration in seconds (`> 0`) |
| `--left-speed` | `0.2` | Left motor speed magnitude in `(0, 0.5]` |
| `--right-speed` | `0.2` | Right motor speed magnitude in `(0, 0.5]` |
| `-t`, `--times` | `1` | Number of interactive trials to run |
| `-p`, `--port` | `/dev/ttyUSB0` | Serial port device path |

#### Examples

Pivot left for `0.5` seconds at default speed (`0.2`):

```bash
python3 scripts/turn.py left
```

Pivot right for `1.0` second at speed `0.3` across `3` trials:

```bash
python3 scripts/turn.py right 1.0 --left-speed 0.3 --right-speed 0.3 -t 3
```

Pivot left with asymmetric wheel speeds (`L=0.25`, `R=0.28`) for `1.5` seconds:

```bash
python3 scripts/turn.py left 1.5 --left-speed 0.25 --right-speed 0.28
```
