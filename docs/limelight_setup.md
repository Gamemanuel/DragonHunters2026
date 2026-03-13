# Vision Camera Setup Guide – Team 10380 (2026 REBUILT Season)

This guide covers two supported configurations for the flywheel vision system:

1. **PhotonVision with a USB camera** (primary – supports FTC-native cameras)
2. **Limelight 3A via NetworkTables** (alternative – see [Section 5](#5-alternative-limelight-3a-via-networktables))

The robot code uses `photonlibpy`, which abstracts the camera connection type
so the same code works regardless of whether the camera connects via USB or
Ethernet.

---

## 1. Hardware Requirements

### Option A – USB Camera + PhotonVision Coprocessor (Recommended)

| Item | Notes |
|------|-------|
| USB camera | Any UVC-compliant camera; FTC-native cameras are typically UVC-compatible |
| Coprocessor | Raspberry Pi 4 (2 GB+), Orange Pi 5, or similar Linux SBC |
| Power (5 V, 3 A+) | Power the coprocessor from the robot's 5 V rail or a dedicated regulator |
| USB cable | Connect camera to coprocessor |
| Ethernet cable | Connect coprocessor to robot radio or switch |

### Option B – Limelight 3A (USB connection to roboRIO)

| Item | Notes |
|------|-------|
| Limelight 3A | USB-C to roboRIO 2.0 USB-A port |
| Power (6–12 V) | Wire from PDP/PDH (fused at 5 A) – separate from USB |

> **Note**: When the Limelight is connected via USB-C to the roboRIO it
> creates a USB virtual network interface (IP `172.22.11.2`).  It still
> publishes to NetworkTables as a client.  This means the robot code works
> the same way as with an Ethernet connection – no code changes are needed
> for USB vs Ethernet.

---

## 2. PhotonVision Coprocessor Setup

### 2.1 Flash PhotonVision

1. Download the latest PhotonVision release image for your coprocessor from
   [github.com/PhotonVision/photonvision/releases](https://github.com/PhotonVision/photonvision/releases).
2. Flash to a microSD card (Raspberry Pi) or eMMC (Orange Pi) with Balena Etcher.
3. Boot the coprocessor and connect it to the same network as the robot radio.

### 2.2 Initial web UI access

1. Connect a laptop to the robot network.
2. Navigate to `http://photonvision.local:5800` or `http://<coprocessor-ip>:5800`.
3. The PhotonVision web interface should appear.

### 2.3 Network configuration

1. In the web UI go to **Settings → Networking**.
2. Set:
   - **Team Number**: `10380`
   - **IP Assignment**: Static
   - **IP Address**: `10.103.80.11` (or another free address on `10.103.80.x`)
   - **Netmask**: `255.255.255.0`
   - **Gateway**: `10.103.80.1`
3. Click **Save** and reboot the coprocessor.

---

## 3. Camera and AprilTag Pipeline Configuration

### 3.1 Add the camera

1. Plug the USB camera into the coprocessor.
2. In the PhotonVision web UI the camera should appear automatically under
   **Cameras**.
3. Click on the camera and rename it to match `PHOTON_CAMERA_NAME` in
   `constants.py` (default: `"photonvision"`).

### 3.2 Create an AprilTag pipeline

1. With the camera selected, click **+ Add Pipeline**.
2. Select **AprilTag** as the pipeline type.
3. Name it `hub` (or any name – the camera name is what matters in code).
4. In the pipeline settings:
   - **Tag Family**: `Tag36h11`
   - **Field Layout**: select or upload `2026_REBUILT.json`
     (available at `<WPILib install>/share/apriltag/2026_REBUILT.json`
     after installing the 2026 WPILib update)
5. Enable **Do Multi-Target Estimation** for the most accurate pose when
   multiple hub tags are visible simultaneously.

### 3.3 Hub AprilTag IDs (verify against official game manual)

> ⚠️ **Important**: Confirm these IDs against the official 2026 REBUILT field
> layout before competition.  The values in `constants.py` default to the IDs
> shown below and must match the field layout JSON.

| Alliance | Tag IDs |
|----------|---------|
| Red hub  | 3, 4   |
| Blue hub | 7, 8   |

In `constants.py`:

```python
HUB_APRILTAG_IDS_RED:  list = [3, 4]
HUB_APRILTAG_IDS_BLUE: list = [7, 8]
```

### 3.4 Camera mounting

- **Height**: ~0.5 m above the floor (adjust to maximize hub tag visibility).
- **Tilt (pitch)**: Angled slightly upward so hub tags are centred in the
  frame at the expected shooting distance range.
- Record the exact mount height and tilt – PhotonVision's 3D pose estimation
  requires accurate camera transform values.

### 3.5 Set camera transform in PhotonVision

In the web UI under **3D** settings, enter the camera's position and rotation
relative to the robot centre:

- **X** (forward), **Y** (left), **Z** (up) in meters
- **Roll**, **Pitch**, **Yaw** in degrees

These values directly affect the accuracy of distance measurements returned to
the robot code.

---

## 4. Robot Code Integration

The `FlywheelSubsystem` uses `photonlibpy` to read AprilTag detections.

```python
from photonlibpy import PhotonCamera
self._camera = PhotonCamera("photonvision")  # must match PHOTON_CAMERA_NAME
```

Per-cycle in `periodic()`, `_get_distance_to_hub()` queries the latest result:

```python
result = self._camera.getLatestResult()
if result.hasTargets():
    for target in result.getTargets():
        if target.getFiducialId() in hub_ids:
            translation = target.getBestCameraToTarget().translation()
            distance = math.hypot(translation.X(), translation.Y())
```

### Verifying connection from the Driver Station

Open the **NetworkTables** view in Shuffleboard or SmartDashboard and look for
the `photonvision/<camera-name>` table.  If targets are visible you will see
`hasTarget = true` and non-zero `latencyMillis`.

### Useful SmartDashboard keys during calibration

| Key | Value |
|-----|-------|
| `Flywheel/Enabled` | Whether the system is active |
| `Flywheel/TargetVelocity_RPS` | Calculated target speed |
| `Flywheel/ActualVelocity_RPS` | Measured motor speed |
| `Flywheel/AtSpeed` | True when within ±2 RPS of target |

---

## 5. Alternative: Limelight 3A via NetworkTables

If you switch to using a Limelight 3A with its native firmware instead of
PhotonVision, you can uncomment the Limelight NT block in
`FlywheelSubsystem.__init__` and `_get_distance_to_hub()`, and comment out the
`PhotonCamera` lines.  Also set `LIMELIGHT_TABLE_NAME` in `constants.py`.

### Limelight network setup (Ethernet or USB)

| Connection | Limelight IP | Notes |
|------------|-------------|-------|
| Ethernet | `10.103.80.11` | Set via Limelight web UI (Settings → Networking) |
| USB-C to roboRIO | `172.22.11.2` | USB gadget mode; NT still works via USB network |

Access the web UI at `http://<limelight-ip>:5801`.

### Limelight AprilTag pipeline

1. Create an **AprilTag** pipeline (index 0).
2. Set **Tag Family** to `36h11` and upload the 2026 REBUILT field JSON.
3. Enable **Multi-tag pose estimation**.
4. The subsystem reads `tv`, `tid`, and `botpose_targetspace` from the
   `limelight` NetworkTable.

---

## 6. Distance-to-Velocity Calibration

The flywheel subsystem uses a linear model:

```
velocity_rps = FLYWHEEL_VELOCITY_SLOPE * distance_m + FLYWHEEL_VELOCITY_INTERCEPT
```

### Calibration procedure

1. Place the robot at a known distance from the hub (e.g. 1 m, 2 m, 3 m, 4 m, 5 m).
2. Enable the flywheel (toggle button / X on the tool controller).
3. Fire a game piece and note whether it hits the hub target.
4. Adjust `FLYWHEEL_VELOCITY_SLOPE` and `FLYWHEEL_VELOCITY_INTERCEPT` in
   `constants.py` until shots are on target across the expected range.
5. Once linear tuning is satisfactory, uncomment the **quadratic** or **lookup
   table** block in `FlywheelSubsystem._calculate_velocity()` for higher accuracy
   at the extremes of the range.

---

## 7. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No targets in PhotonVision | Camera not connected / pipeline not set | Check USB cable, select AprilTag pipeline |
| `hasTarget` never true | Wrong camera name | Ensure `PHOTON_CAMERA_NAME` matches web UI name |
| Wrong tag ID detected | Another field tag in view | Narrow FOV or filter by ID (done automatically in code) |
| Distance reads ~0 | Camera transform not set | Enter mount position in PhotonVision 3D settings |
| Flywheel speed never changes | Camera name mismatch or no valid target | Check SmartDashboard for `Flywheel/TargetVelocity_RPS` |
| `AtSpeed` never true | PID gains need tuning | Adjust `FLYWHEEL_KP`, `FLYWHEEL_KV`, `FLYWHEEL_KS` |

---

## 8. References

- [PhotonVision documentation](https://docs.photonvision.org)
- [photonlibpy API](https://pypi.org/project/photonlibpy/)
- [WPILib AprilTag field layouts](https://github.com/wpilibsuite/allwpilib/tree/main/apriltag)
- [Phoenix 6 VelocityVoltage API](https://api.ctr-electronics.com/phoenix6/release/python/)
- 2026 REBUILT official game manual (FIRST website)

