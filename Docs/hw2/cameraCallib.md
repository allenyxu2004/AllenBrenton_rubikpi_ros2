# Camera Calibration Guide (HW2)

## Checkerboard Specifications
- **Paper Size:** 8.5" x 11"
- **Grid Squares:** $9 \times 7$ grid
- **Interior Corners (`--size`):** $8 \times 6$ ($(9-1) \times (7-1)$)
- **Square Size (`--square`):** $1.75\text{ cm} \times 1.75\text{ cm} = \mathbf{0.0175\text{ m}}$

---

## Method 1: Live GUI Calibration (`camera_calibration` + VcXsrv)

### 1. Start VcXsrv (`XLaunch`) on Windows
1. Open **XLaunch** from the Start Menu.
2. Select **Multiple windows** $\rightarrow$ **Next**.
3. Select **Start no client** $\rightarrow$ **Next**.
4. Check **Disable access control** $\rightarrow$ **Next** $\rightarrow$ **Finish**.

### 2. SSH into the RubikPi with X11 Forwarding
In Windows PowerShell, set `DISPLAY` and connect with `-Y -C` (X11 forwarding + compression):

```powershell
$env:DISPLAY = "localhost:0.0"
ssh -Y -C ubuntu@<RUBIKPI_IP>
```

Verify X11 forwarding is active on the RubikPi:

```bash
echo $DISPLAY
# Should output something like: localhost:10.0
```

*(If `echo $DISPLAY` is empty, install `sudo apt install -y xauth` on the RubikPi, then `exit` and reconnect).*

### 3. Launch the Camera Node (Unrectified)
In **Terminal 1** on the RubikPi, start the camera node with rectification disabled:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch robot_vision_camera robot_vision_camera.launch.py image_rectify:=false
```

### 4. Run the ROS 2 Camera Calibrator GUI
In **Terminal 2** (your `ssh -Y -C` session) on the RubikPi:

```bash
# Install if not already installed:
sudo apt install -y ros-jazzy-camera-calibration

# Force Qt to use X11 (VcXsrv) instead of Wayland:
export QT_QPA_PLATFORM=xcb
unset WAYLAND_DISPLAY

# Launch the calibrator GUI:
source /opt/ros/jazzy/setup.bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.0175 --no-service-check --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

### 5. Calibrate & Extract Parameters
1. Move and tilt the checkerboard in front of the camera until the **X**, **Y**, **Size**, and **Skew** bars turn green and **CALIBRATE** lights up.
2. Click **CALIBRATE**, wait for computation to finish, then click **SAVE**.
3. Extract the generated calibration archive on the RubikPi:
   ```bash
   mkdir -p /tmp/calib
   tar -xzf /tmp/calibrationdata.tar.gz -C /tmp/calib
   cat /tmp/calib/ost.yaml
   ```
4. Copy the `data: [...]` arrays (`camera_matrix`, `distortion_coefficients`, `rectification_matrix`, `projection_matrix`) from `/tmp/calib/ost.yaml` into `robot_vision/config/camera_parameter.yaml` (keeping the `%YAML:1.0` header and `!!opencv-matrix` tags intact).

---

## Step 6: Rebuild and Launch with Rectification Enabled

After updating `robot_vision/config/camera_parameter.yaml`, rebuild `robot_vision_camera` on the RubikPi so the installed config updates, then launch with `image_rectify:=true`:

```bash
cd ~/ros2_ws
colcon build --packages-select robot_vision_camera
source install/setup.bash
ros2 launch robot_vision_camera robot_vision_camera.launch.py image_rectify:=true
```
